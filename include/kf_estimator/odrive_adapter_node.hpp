#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <odrive_can/msg/controller_status.hpp> // Assumes this message is available
#include <map>
#include <vector>
#include <string>

using odrive_can::msg::ControllerStatus;
using sensor_msgs::msg::JointState;

// Define the 8 active joints based on the XML structure (Head=Front, Torso=Rear)
struct JointInfo {
    std::string topic_name;
    std::string joint_name;
    // Data fields to store the latest estimates
    double pos = 0.0;
    double vel = 0.0;
    double eff = 0.0;
    bool received = false; // Flag to ensure data has been received at least once
};

class OdriveJointStateAdapter : public rclcpp::Node {
public:
    OdriveJointStateAdapter();

private:
    // This method is no longer needed because the logic is now in a lambda
    // void status_callback(const ControllerStatus::SharedPtr msg, const std::string& joint_key);
    void publish_joint_state();

    // The order of joints in this vector determines the order in the published JointState message
    std::vector<std::string> joint_keys_ = {
        "fl_hip", "fl_knee", 
        "fr_hip", "fr_knee", 
        "rl_hip", "rl_knee", 
        "rr_hip", "rr_knee"
    };

    // Maps the simplified key (e.g., "fl_hip") to its full data structure
    std::map<std::string, JointInfo> joint_data_;

    // Map to hold the 8 subscribers
    std::map<std::string, rclcpp::Subscription<ControllerStatus>::SharedPtr> subscriptions_;

    rclcpp::Publisher<JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Static array of final output joint names (matching the EKF's assumed order from XML)
    const std::vector<std::string> output_joint_names_ = {
        "head_left_thigh_joint",     // FL-Hip
        "head_left_thigh_shin_joint",// FL-Knee
        "head_right_thigh_joint",    // FR-Hip
        "head_right_thigh_shin_joint",// FR-Knee
        "torso_left_thigh_joint",    // RL-Hip
        "torso_left_thigh_shin_joint",// RL-Knee
        "torso_right_thigh_joint",   // RR-Hip
        "torso_right_thigh_shin_joint" // RR-Knee
    };
};