#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "rmw/qos_profiles.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include <mutex>

class FindWallNode : public rclcpp::Node {
public:
    FindWallNode()
        : Node("finde_wall_service_node"), rate_(20), laser_message_count_(0){
        
        // Create a reentrant callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        
        // Subscription options 
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        // Initialize publisher and subscriber
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10,
            std::bind(&FindWallNode::laser_callback, this, std::placeholders::_1),
            options
        );

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize service with callback group
        service_ = this->create_service<custom_interfaces::srv::FindWall>(
            "/find_wall",
            std::bind(&FindWallNode::service_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );

        cmd_vel_msg = geometry_msgs::msg::Twist();
    }

private:
    // Class variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::FindWall>::SharedPtr service_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Rate rate_;

    std::mutex laser_data_mutex_;  // Using mutex to avoid conflicting access to laser_data_

    geometry_msgs::msg::Twist cmd_vel_msg;
    sensor_msgs::msg::LaserScan::SharedPtr laser_data_;  // SharedPtr for laser data

    // Server corret synchronization control variables
    int laser_message_count_;                   // Count of laser data received
    const int min_laser_messages_needed_ = 10;  // Min number of laser data needed
    
    // Get the minimum value and index from laser ranges
    std::pair<float, size_t> get_min_range(const std::vector<float> &ranges) {
        float min_range = std::numeric_limits<float>::infinity();
        size_t min_range_index = 0;

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (std::isfinite(ranges[i]) && ranges[i] < min_range) {
                min_range = ranges[i];
                min_range_index = i;
            }
        }
        return std::make_pair(min_range, min_range_index);
    }

    // Laser scan subscriber callback
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(laser_data_mutex_); // Lock mutex to safely modify laser_data_
        laser_data_ = msg;  // Update the laser to get the latest msg
        
        laser_message_count_++; // Increment the count of received messages
    }

    void service_callback(const std::shared_ptr<custom_interfaces::srv::FindWall::Request> request,
                          std::shared_ptr<custom_interfaces::srv::FindWall::Response> response) {
        (void)request;
        
        RCLCPP_INFO(this->get_logger(), "The service '/find_wall' has been called...");

        // Service client communication safty 

        // Ensue laser data is available 
        while (rclcpp::ok() &&  laser_message_count_ < min_laser_messages_needed_){
            RCLCPP_INFO(this->get_logger(), "Waiting for sufficient laser data...");
            rclcpp::sleep_for(std::chrono::milliseconds(100));  // Deley  
        }
        
        // Check again after witing to ensure data is rady 
        if (laser_message_count_ < min_laser_messages_needed_) {
            RCLCPP_ERROR(this->get_logger(), "Insufficent laser data received");
            response->wallfound = false;
            return;
        }

        // Start //

        // Safely copy the shared laser_data_ and make a local copy to release the mutex
        sensor_msgs::msg::LaserScan::SharedPtr local_laser_data_;
        {
            std::lock_guard<std::mutex> lock(laser_data_mutex_);
            local_laser_data_ = laser_data_;
        }
        
        
        if (!local_laser_data_ || local_laser_data_->ranges.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No laser data available");
            response->wallfound = false;
            return;
        }

        // Calculate fixed indexes
        size_t front_index_ = static_cast<size_t>((0.0 - local_laser_data_->angle_min) / local_laser_data_->angle_increment);  // Front index (0)
        size_t right_index_ = static_cast<size_t>((3 * M_PI / 2 - local_laser_data_->angle_min) / local_laser_data_->angle_increment);  // Right index (270)

        // Get angles for comparison
        float front_laser_angle_ = local_laser_data_->angle_min + front_index_ * local_laser_data_->angle_increment;

        RCLCPP_INFO(this->get_logger(), "Finding closest wall...");
        
        // 1. Rotate until front laser ray is aligned with closest wall
        while (rclcpp::ok()) {
            // Get the latest laser data in each loop iteration
            {
                std::lock_guard<std::mutex> lock(laser_data_mutex_);
                local_laser_data_ = laser_data_;
            }

            // Find the minimum range and its index
            auto min_value_pair = get_min_range(local_laser_data_->ranges);
            float min_value_angle = local_laser_data_->angle_min + min_value_pair.second * local_laser_data_->angle_increment;

            // Check alignment within tolerance
            if (std::abs(front_laser_angle_ - min_value_angle) <= 0.05) {
                break; // Aligned
            }

            // Rotate the robot slowly
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.1;
            publisher_->publish(cmd_vel_msg);

            rate_.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Wall was found...");

        // Stop rotation
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        publisher_->publish(cmd_vel_msg);

        RCLCPP_INFO(this->get_logger(), "Moving forward to the wall...");

        // 2. Move forward until front laser range is less than 0.3m
        while (rclcpp::ok()) {
            // Get the latest laser data in each loop iteration
            {
                std::lock_guard<std::mutex> lock(laser_data_mutex_);
                local_laser_data_ = laser_data_;
            }

            if (local_laser_data_->ranges[front_index_] <= 0.3) {
                break; // Stop when front range is less than 0.3m
            }

            cmd_vel_msg.linear.x = 0.05;
            cmd_vel_msg.angular.z = 0.0;
            publisher_->publish(cmd_vel_msg);

            rate_.sleep();
        }

        // Stop forward movement
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        publisher_->publish(cmd_vel_msg);
        
        RCLCPP_INFO(this->get_logger(), "Alining parale to the wall...");

        // 3. Rotate until right laser ray is aligned with the closest wall
        float right_laser_angle_ = local_laser_data_->angle_min + right_index_ * local_laser_data_->angle_increment;

        while (rclcpp::ok()) {
            // Get the latest laser data in each loop iteration
            {
                std::lock_guard<std::mutex> lock(laser_data_mutex_);
                local_laser_data_ = laser_data_;
            }

            auto min_value_pair = get_min_range(local_laser_data_->ranges);
            float min_value_angle = local_laser_data_->angle_min + min_value_pair.second * local_laser_data_->angle_increment;

            float angle_error_ = min_value_angle - right_laser_angle_;

            // Normalize the angle difference to be within the range (-π, π)
            while (angle_error_ > M_PI) angle_error_ -= 2 * M_PI;
            while (angle_error_ < -M_PI) angle_error_ += 2 * M_PI;

            // Check alignment within tolerance
            if (std::fabs(angle_error_) <= 0.05) {
                break; // Aligned
            }

            //RCLCPP_INFO(this->get_logger(), "right_laser_angle_: %f, min_value_angle: %f, angle_error_: %f", right_laser_angle_, min_value_angle, angle_error_);

            // Rotate the robot slowly
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.1 * angle_error_;
            publisher_->publish(cmd_vel_msg);

            rate_.sleep();
        }

        // Stop rotation
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        publisher_->publish(cmd_vel_msg);

        RCLCPP_INFO(this->get_logger(), "Ready to start wall following behavior...");

        // Return the response indicating the wall was found and aligned
        response->wallfound = true;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<FindWallNode>();

    // Create a MultiThreadedExecutor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin the executor
    executor.spin();

    rclcpp::shutdown();
    return 0;
}