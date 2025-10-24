#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int8_multi_array.hpp"

#include "osc_2_in_interface/msg/osc_mujoco_state.hpp" 

#include <chrono>
#include <vector>
#include <cmath> 
#include <cstdint>      //  FIX 1: Required for standard integer types like uint64_t
#include <algorithm>    //  FIX 2: Required for std::fill
#include <array>        // Required for std::array usage

using namespace std::chrono_literals;

// Define robot constants
const size_t NUM_JOINTS = 8; // Changed to size_t for comparison safety
const size_t NUM_FEET = 8;
const double PUBLISH_FREQUENCY = 100.0; // Hz

// Alias for the new message type
using OSCMujocoState = osc_2_in_interface::msg::OSCMujocoState;

class StateConverterNode : public rclcpp::Node
{
public:
    StateConverterNode() : Node("state_converter_node")
    {
        // Initialize latest data storage
        last_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
        last_imu_data_ = std::make_shared<sensor_msgs::msg::Imu>();
        last_contact_mask_.assign(NUM_FEET, false);
        is_initialized_ = false;

        // Subscriptions
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states_in", 10,
            std::bind(&StateConverterNode::joint_state_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data_raw", 10,
            std::bind(&StateConverterNode::imu_data_callback, this, std::placeholders::_1));

        contact_sub_ = this->create_subscription<std_msgs::msg::Int8MultiArray>(
            "contact_schedule", 10,
            std::bind(&StateConverterNode::contact_callback, this, std::placeholders::_1));

        // Publisher
        state_pub_ = this->create_publisher<OSCMujocoState>("state_estimator/state", 10);

        // Timer for publishing state at a fixed rate
        timer_ = this->create_wall_timer(
            1s / PUBLISH_FREQUENCY,
            std::bind(&StateConverterNode::publish_state, this));
            
        RCLCPP_INFO(this->get_logger(), "State Converter Node Initialized.");
    }

private:
    // --- Data Storage ---
    sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
    sensor_msgs::msg::Imu::SharedPtr last_imu_data_;
    std::vector<bool> last_contact_mask_;
    bool is_initialized_;

    // --- ROS 2 Members ---
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8MultiArray>::SharedPtr contact_sub_;
    rclcpp::Publisher<OSCMujocoState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callbacks ---

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() == NUM_JOINTS) {
            *last_joint_state_ = *msg;
            if (!is_initialized_) is_initialized_ = true;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "JointState size mismatch! Expected %zu, got %zu.", 
                NUM_JOINTS, msg->position.size());
        }
    }

    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        *last_imu_data_ = *msg;
    }

    void contact_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == NUM_FEET) {
            for (size_t i = 0; i < NUM_FEET; ++i) {
                last_contact_mask_[i] = (msg->data[i] != 0);
            }
        }
    }

    // --- Publisher Loop ---

    void publish_state()
    {
        if (!is_initialized_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for joint data to initialize.");
            return;
        }

        auto state_msg = OSCMujocoState();
        
        // --- Header and Timestamp ---
        // Convert ROS 2 Time (sec + nanosec) to uint64 nanoseconds
        state_msg.timestamp = last_joint_state_->header.stamp.sec * 1000000000ULL + last_joint_state_->header.stamp.nanosec;
        
        // --- Joint State Mapping (All are float) ---
        
        if (last_joint_state_->position.size() == NUM_JOINTS && 
            last_joint_state_->velocity.size() == NUM_JOINTS) 
        {
            for (size_t i = 0; i < NUM_JOINTS; ++i) { // 🛠️ FIX 3: Use size_t for loop
                
                // MOTOR POSITIONS & VELOCITIES
                // Note: The float32_t type alias is 'float' in C++
                state_msg.motor_position[i] = (float)last_joint_state_->position[i];
                state_msg.motor_velocity[i] = (float)last_joint_state_->velocity[i];
                
                // MOTOR ACCELERATION & TORQUE ESTIMATE (using effort as proxy)
                if (last_joint_state_->effort.size() > i) {
                    float effort_val = (float)last_joint_state_->effort[i]; // 🛠️ FIX 4: Use 'float' instead of 'float32_t'
                    state_msg.motor_acceleration[i] = effort_val;
                    state_msg.torque_estimate[i] = effort_val;
                } else {
                    state_msg.motor_acceleration[i] = 0.0f;
                    state_msg.torque_estimate[i] = 0.0f;
                }
            }
        }

        // --- Base/IMU State Mapping (All float) ---
        
        // BODY ROTATION (w, x, y, z)
        state_msg.body_rotation[0] = (float)last_imu_data_->orientation.w;
        state_msg.body_rotation[1] = (float)last_imu_data_->orientation.x;
        state_msg.body_rotation[2] = (float)last_imu_data_->orientation.y;
        state_msg.body_rotation[3] = (float)last_imu_data_->orientation.z;

        // ANGULAR BODY VELOCITY (Roll_dot, Pitch_dot, Yaw_dot)
        state_msg.angular_body_velocity[0] = (float)last_imu_data_->angular_velocity.x;
        state_msg.angular_body_velocity[1] = (float)last_imu_data_->angular_velocity.y;
        state_msg.angular_body_velocity[2] = (float)last_imu_data_->angular_velocity.z;
        
        // LINEAR BODY ACCELERATION 
        state_msg.linear_body_acceleration[0] = (float)last_imu_data_->linear_acceleration.x;
        state_msg.linear_body_acceleration[1] = (float)last_imu_data_->linear_acceleration.y;
        state_msg.linear_body_acceleration[2] = (float)last_imu_data_->linear_acceleration.z;

        // LINEAR BODY VELOCITY (Set to 0.0f - Requires std::fill for fixed array)
        std::fill(state_msg.linear_body_velocity.begin(), state_msg.linear_body_velocity.end(), 0.0f); 

        // --- Contact Mask ---
        for (size_t i = 0; i < NUM_FEET; ++i) {
            state_msg.contact_mask[i] = last_contact_mask_[i];
        }

        // Publish the message
        state_pub_->publish(state_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateConverterNode>());
    rclcpp::shutdown();
    return 0;
}