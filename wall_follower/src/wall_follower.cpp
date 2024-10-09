#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <functional>
#include <limits> 
#include <future>
#include <memory>

#include "custom_interfaces/srv/find_wall.hpp"
#include "custom_interfaces/action/odom_record.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"

using namespace std::chrono_literals;

class DriveAlongWall : public rclcpp::Node {
  public:    
    using OdomRecord = custom_interfaces::action::OdomRecord;

    DriveAlongWall()
        : Node("wall_follower_node"), wall_found_(false), odometry_recorder_started_(false) {
        // Initialize publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Initilaize subscriber
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DriveAlongWall::laser_callback, this, std::placeholders::_1));

        // Initialize the Twist message
        twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();

        // Create services clinet for find_wall_service
        find_wall_client_ = this->create_client<custom_interfaces::srv::FindWall>("/find_wall");
        
        // Create action clinet for the odom_record_action_server 
        odometry_client_ = rclcpp_action::create_client<custom_interfaces::action::OdomRecord>(this, "/record_odom");    
        
        // Set the loop rate
        timer_ = this->create_wall_timer(
            200ms, std::bind(&DriveAlongWall::timer_callback, this)
        );
        
    }
    
    void execute_sequence() {
        // Call the service to find the wall
        if (call_find_wall_service()) {
            RCLCPP_INFO(this->get_logger(), "Wall found successfully. Proceeding to call odom_record_action_server.");
            
            // Call the odom_record_action_server
            if (call_odom_record_action_server()) {
                RCLCPP_INFO(this->get_logger(), "Odometry recording started successfully.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to start odometry recording.");
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to call the find_wall service.");
        }
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Client<custom_interfaces::srv::FindWall>::SharedPtr find_wall_client_;
    rclcpp_action::Client<OdomRecord>::SharedPtr odometry_client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<geometry_msgs::msg::Twist> twist_msg_;
    
    bool wall_found_; // flag for server
    bool odometry_recorder_started_; // flag for action 
    float total_distance_;
    
    // Function to call find_wall Service
    bool call_find_wall_service() {
        const int max_attemps = 10; 
        int attempts = 0;

        // Wait for the service to be available 
        while (!find_wall_client_->wait_for_service(5s)) {    
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
        auto result_future_ = find_wall_client_->async_send_request(request_);
        
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

    // Function to call odom_record_action_server
    bool call_odom_record_action_server() {
        // Wait for the action server to become available 
        if (!odometry_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "/record_odom action server is not available.");
            return false;
        }

        // Create a goal message (empty since action doesn't require one)
        auto goal_msg = OdomRecord::Goal();

        // Defie the goal options        
        rclcpp_action::Client<OdomRecord>::SendGoalOptions goal_options;
        goal_options.goal_response_callback = 
            std::bind(&DriveAlongWall::goal_response_callback, this, std::placeholders::_1);
        goal_options.feedback_callback = 
            std::bind(&DriveAlongWall::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        goal_options.result_callback = 
            std::bind(&DriveAlongWall::result_callback, this, std::placeholders::_1);

        // Send the goal asynchronously
        auto goal_handle_future = odometry_client_->async_send_goal(goal_msg, goal_options);
        
        // Wait for goal acceptance
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send goal for odometry recording");       
            return false;
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Goal was accepted by the action server"); 
        return true;
    }

    // Callback for goal respons
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<OdomRecord>::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(),"Odometry recording goal was rejected");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Odometry recording goal was accepted.");
            odometry_recorder_started_ = true;
            
        }
    }

    // Callback for feedback response
    void feedback_callback(rclcpp_action::ClientGoalHandle<OdomRecord>::SharedPtr,
                           const std::shared_ptr<const OdomRecord::Feedback> feedback) {

        total_distance_ = feedback->current_total;    
        RCLCPP_INFO(this->get_logger(), "Odometry recording in progress... Total distance recorded: %.2f meters", total_distance_);
    } 

    void result_callback(const rclcpp_action::ClientGoalHandle<OdomRecord>::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Odometry recording succeeded.");
                odometry_recorder_started_ = false; // Reset the flag as action is complete and stop movement 
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Odometry recording was aborted.");
                odometry_recorder_started_ = false;                
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Odometry recording was canceled.");
                odometry_recorder_started_ = false;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code for odometry.");
                odometry_recorder_started_ = false;
                break;
        }
    }

    // Wall following behavior callback 
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
        // RCLCPP_INFO(this->get_logger(), "Indices - Right: %d, Front: %d, Left: %d", right_index, front_index, left_index);
        // RCLCPP_INFO(this->get_logger(), "Ranges - Right: %f, Front: %f, Left: %f", right, front, left);

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
        if (wall_found_ && odometry_recorder_started_) {
            publisher_->publish(*twist_msg_);
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveAlongWall>();
    
    // Execute the sequence 
    node->execute_sequence();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}