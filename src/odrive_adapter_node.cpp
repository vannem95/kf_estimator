#include "kf_estimator/odrive_adapter_node.hpp"
#include <functional> // Still good practice to include but we're moving away from std::bind

OdriveJointStateAdapter::OdriveJointStateAdapter()
    : Node("odrive_joint_state_adapter") {

    RCLCPP_INFO(this->get_logger(), "Starting ODrive Joint State Adapter Node.");

    // --- 1. Initialize Joint Data and Subscribers ---
    for (size_t i = 0; i < joint_keys_.size(); ++i) {
        const std::string& key = joint_keys_[i];
        
        // Populate JointInfo structure
        joint_data_[key].topic_name = "/" + key + "/controller_status";
        joint_data_[key].joint_name = output_joint_names_[i];

        // Create the subscriber for the joint's status topic
        subscriptions_[key] = this->create_subscription<ControllerStatus>(
            joint_data_[key].topic_name,
            10,
            // Replace std::bind with a lambda function
            [this, key](const ControllerStatus::SharedPtr msg) {
                if (joint_data_.count(key)) {
                    // Update latest data
                    joint_data_[key].pos = msg->pos_estimate;
                    joint_data_[key].vel = msg->vel_estimate;
                    joint_data_[key].eff = msg->torque_estimate;
                    joint_data_[key].received = true;
                }
            }
        );
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", joint_data_[key].topic_name.c_str());
    }

    // The rest of the constructor remains the same
    joint_state_pub_ = this->create_publisher<JointState>("joint_states_in", 10);
    
    double publish_rate_hz = 100.0;
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz),
        std::bind(&OdriveJointStateAdapter::publish_joint_state, this));
}

// The old status_callback function is no longer needed
// void OdriveJointStateAdapter::status_callback(const ControllerStatus::SharedPtr msg, const std::string& joint_key) {
//     if (joint_data_.count(joint_key)) {
//         joint_data_[joint_key].pos = msg->pos_estimate;
//         joint_data_[joint_key].vel = msg->vel_estimate;
//         joint_data_[joint_key].eff = msg->torque_estimate;
//         joint_data_[joint_key].received = true;
//     }
// }

void OdriveJointStateAdapter::publish_joint_state() {
    bool all_received = true;
    for (const auto& pair : joint_data_) {
        if (!pair.second.received) {
            all_received = false;
            break;
        }
    }

    if (!all_received) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for initial data from all 8 ODrive controllers...");
        return;
    }

    auto msg = JointState();
    msg.header.stamp = this->now();
    
    for (const std::string& key : joint_keys_) {
        const auto& data = joint_data_[key];
        msg.name.push_back(data.joint_name);
        msg.position.push_back(data.pos);
        msg.velocity.push_back(data.vel);
        msg.effort.push_back(data.eff); 
    }

    joint_state_pub_->publish(msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdriveJointStateAdapter>());
    rclcpp::shutdown();
    return 0;
}