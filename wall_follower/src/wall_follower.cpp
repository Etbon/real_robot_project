#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <limits> 

#include "custom_interfaces/srv/detail/find_wall__struct.hpp"
#include "custom_interfaces/srv/find_wall.hpp"
#include "rclcpp/client.hpp"

using namespace std::chrono_literals;

class DriveAlongWall : public rclcpp::Node {
  public:
    DriveAlongWall()
        : Node("wall_follower_node"), wall_found_(false) {
        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initilaize subscriber
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DriveAlongWall::laser_callback, this, std::placeholders::_1));

        // Initialize the Twist message
        twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();

        // Create services clinet for find_wall_service
        client_ = this->create_client<custom_interfaces::srv::FindWall>("/find_wall");
            
        // Set the loop rate
        timer_ = this->create_wall_timer(
            100ms, std::bind(&DriveAlongWall::timer_callback, this)
        );
        
        // Call the service before starting the wall_following behavior 
        if (call_find_wall_service()) {
            // Continue with program starts the fallowing-wall-behevier  
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to find wall");
        }
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Client<custom_interfaces::srv::FindWall>::SharedPtr client_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<geometry_msgs::msg::Twist> twist_msg_;
    
    bool wall_found_;

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        if(!wall_found_) {
            //RCLCPP_INFO(this->get_logger(), "Waiting for wall detection before processing laser data...");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Laser callback triggered.");

        // Ensure the indices are within bounds
        if (msg->ranges.size() < 360) {
            RCLCPP_ERROR(this->get_logger(), "LaserScan message does not have enough data.");
            return;
        }

        // Define the ranges for right, front, and left
        const int right_index = 180;
        const int front_index = 360;
        const int left_index = 540;

        // Function to find the minimum value in a given range, ignoring inf values
        auto get_min_range = [](const std::vector<float>& ranges, int start_index, int end_index) -> float {
            float min_value = std::numeric_limits<float>::infinity();
            for (int i = start_index; i <= end_index; ++i) {
                if (std::isfinite(ranges[i])) {
                    min_value = std::min(min_value, ranges[i]);
                }
            }
            return min_value;
        };

        // Get the minimum values for right, front, and left
        float right = get_min_range(msg->ranges, right_index - 10, right_index + 10);
        float front = get_min_range(msg->ranges, front_index - 10, front_index + 10);
        float left = get_min_range(msg->ranges, left_index - 10, left_index + 10);

        // Add diagnostic information
        RCLCPP_INFO(this->get_logger(), "Indices - Right: %d, Front: %d, Left: %d", right_index, front_index, left_index);
        RCLCPP_INFO(this->get_logger(), "Ranges - Right: %f, Front: %f, Left: %f", right, front, left);

        // Wall-following logic
        if (front < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Turn fast to left");
            twist_msg_->linear.x = 0.1;
            twist_msg_->angular.z = 1;
        } 
        else if (right > 0.3) {
            RCLCPP_INFO(this->get_logger(), "Find the wall");
            twist_msg_->linear.x = 0.1;
            twist_msg_->angular.z = -0.1;
        }
        else if (right < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Move away from the wall");
            twist_msg_->linear.x = 0.05;
            twist_msg_->angular.z = 0.1;
        } 
        else if (right > 0.2 && right < 0.3) {
            RCLCPP_INFO(this->get_logger(), "Follow the wall");
            twist_msg_->linear.x = 0.1;
            twist_msg_->angular.z = 0.0;
        }
        else if (left < 0.3) {
            RCLCPP_INFO(this->get_logger(), "Move away from obstacle");
            twist_msg_->linear.x = 0.0;
            twist_msg_->angular.z = -0.1;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Stop");
            twist_msg_->linear.x = 0.0;
            twist_msg_->angular.z = 0.0;
        }
    }

    void timer_callback() {
        if (wall_found_) {
            publisher_->publish(*twist_msg_);
        }
    }
    
    // Funtion to call find_wall Service
    bool call_find_wall_service() {
        const int max_attemps = 10; 
        int attempts = 0;

        // Wait for the service to be available 
        while (!client_->wait_for_service(5s)) {
            
            RCLCPP_INFO(this->get_logger(), "Waiting for the '/find_wall' service...");
            
            attempts++;
            
            if(attempts >= max_attemps) {
                RCLCPP_ERROR(this->get_logger(), "Max attemps reached Could not find wall '/find_wall' services.");
                return false;
            }

        }
        
        // Create a request to send to the service 
        auto request_ = std::make_shared<custom_interfaces::srv::FindWall::Request>();

        // Call the service asynchronously
        auto result_future_ = client_->async_send_request(request_);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future_) == 
            rclcpp::FutureReturnCode::SUCCESS) {

            auto result = result_future_.get();
            if (result->wallfound) {
                RCLCPP_INFO(this->get_logger(), "finde_wall service completed successfully. Ready to follow the wall.");
                wall_found_ = true;
                return true;
            }
            else {
                RCLCPP_ERROR(this->get_logger(), "The find_wall service returned 'false'. Wall was not found.");
                return false;
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call find_wall service.");
            return false;
        }
        
    }
    
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveAlongWall>());
    rclcpp::shutdown();
    return 0;
}