#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include "custom_interfaces/action/odom_record.hpp"


class OdomRecordActionServer : public rclcpp::Node {
  public:
    using OdomRecord = custom_interfaces::action::OdomRecord;
    using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

    explicit OdomRecordActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("odom_record_action_server_node", options), initialized_(false), total_distance_(0.0) {

        // Initialize callback_group
        odomRecord_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = odomRecord_callback_group_;

        // Initialize the action_server (goal, cancel, accepted)
        this->action_server_ = rclcpp_action::create_server<OdomRecord>(
            this,
            "/record_odom", 
            std::bind(&OdomRecordActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OdomRecordActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&OdomRecordActionServer::handle_accepted, this, std::placeholders::_1));

        // Initialize the QoS (k,b,d)
        rclcpp::QoS qos(10);
        qos.keep_last(10);
        qos.best_effort();
        qos.durability_volatile();

        // Initialize odom_subscriber
        this->odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", qos, std::bind(&OdomRecordActionServer::odom_callback, this, std::placeholders::_1), sub_options);
    }

  private:
    // Define the member variables
    rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::CallbackGroup::SharedPtr odomRecord_callback_group_;

    std::vector<geometry_msgs::msg::Point32> list_of_odoms_;

    bool initialized_;
    float total_distance_;
    double previous_x_, previous_y_;
    double current_x_, current_y_;
    double start_x_, start_y_;

    const double lap_threshold_ = 0.1; // Set a threshold to determine when the robot has complited the lap

    // Handle the goal callback
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const OdomRecord::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle the cancel callback
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle the accepted callback
    void handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
        std::thread{std::bind(&OdomRecordActionServer::executor, this, std::placeholders::_1),
        goal_handle}.detach();
    }
    
    // Odometry callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Get x,y coordinates
        this->current_x_ = msg->pose.pose.position.x; 
        this->current_y_ = msg->pose.pose.position.y;
        
        // Get the first mesurment 
        if (!initialized_) {
            previous_x_ = current_x_;
            previous_y_ = current_y_;
            start_x_ = current_x_;
            start_y_ = current_y_;
            initialized_ = true;
            return;
        }
        measure_distance();
        record_odometry();
    }

    // Measuring function
    void measure_distance() {
        // Calculating distance 
        double d_increment = std::sqrt(std::pow(current_x_ - previous_x_, 2) + std::pow(current_y_ - previous_y_, 2));
        // Adding the total distance 
        total_distance_ += d_increment;

        previous_x_ = current_x_;
        previous_y_ = current_y_;

        RCLCPP_INFO(this->get_logger(), "Total distance: %f", total_distance_);
    }

    void record_odometry() {
        geometry_msgs::msg::Point32 point;
        point.x = current_x_;
        point.y = current_y_;
        list_of_odoms_.push_back(point);
    }

    // Executor function 
    void executor(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto feedback = std::make_shared<OdomRecord::Feedback>();
        auto result = std::make_shared<OdomRecord::Result>();
        rclcpp::Rate loop_rate(1); // 1Hz loop rate

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->list_of_odoms = list_of_odoms_;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            if (is_lap_completed()) {
                result->list_of_odoms = list_of_odoms_;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), " Goal succeeded, odometry data recorded");
                return;
            }

            feedback->current_total = total_distance_;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %d", total_distance_);
            loop_rate.sleep();
        }
    
        result->list_of_odoms = list_of_odoms_;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeede, odometry data recorded");
    }

    bool is_lap_completed() {
        // Calculating if the lap is completed
        double distance_to_start = std::sqrt(std::pow(current_x_ - start_x_, 2) + std::pow(current_y_ - start_y_,2));
        
        if (distance_to_start < lap_threshold_) {
            RCLCPP_INFO(this->get_logger(), "Lap completed");
            return true;
        }
        return false;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomRecordActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}