#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp> // <-- Add this header
#include <std_msgs/msg/int8_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <mujoco/mujoco.h>

#include <Eigen/Dense>

#include <map>
#include <vector>
#include <string>

//  viewer
#include <thread> // Add this for threading


// Use aliases for readability
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Imu; // <-- Add this alias
using std_msgs::msg::Int8MultiArray;
using nav_msgs::msg::Odometry;


// Constants based on your error output hints
const int N_BASE_POS = 3;
const int N_BASE_ORIENT = 4; // Using a quaternion
const int N_BASE_LIN_VEL = 3;
const int N_BASE_ANG_VEL = 3;
const int N_BASE_STATES = N_BASE_POS + N_BASE_ORIENT + N_BASE_LIN_VEL + N_BASE_ANG_VEL;

const int N_JOINT_POS = 8;
const int N_JOINT_VEL = 8;
const int N_JOINT_STATES = N_JOINT_POS + N_JOINT_VEL;

const int N_TOTAL_STATES = N_BASE_STATES + N_JOINT_STATES; // This will now correctly be 29
const int NUM_FEET = 8; // Add this new constant
const double DT = 0.005; // Example timestep, adjust as needed

class QuadrupedEKF {
public:
    // This constructor matches your source file definition
    QuadrupedEKF(double dt, mjModel* model, mjData* data);

    // Update this line to include joint_vel
    void initialize_state(const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_vel);

    void predict(const Vector3d& base_acc, const Vector3d& base_gyro);

    // This method now takes all the correct arguments
    void update(const Eigen::VectorXd& z, const std::vector<bool>& contact_states, mjModel* m_ptr, mjData* d_ptr);

    const Eigen::VectorXd& get_state() const {
        return x_;
    }    
private:
    VectorXd x_; // State vector
    MatrixXd P_; // Covariance matrix
    MatrixXd Q_; // Process noise covariance
    MatrixXd R_; // Measurement noise covariance

    double dt_;
    mjModel* model_ptr_;
    mjData* data_ptr_;

    // Mapping for MuJoCo joint data
    std::map<std::string, int> joint_to_qpos_index_map_;
    
    int foot_vel_sensor_indices[NUM_FEET]; // Store the starting index in d_ptr->sensordata for each foot's velocity    
};

class EKFNode : public rclcpp::Node {
public:
    EKFNode();
    ~EKFNode(); // Destructor to free MuJoCo data

private:
    void estimator_loop();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg); // <-- Add IMU callback
    void contact_schedule_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg);

    // Subscribers and Publishers
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_; // <-- Add IMU subscriber
    rclcpp::Subscription<Int8MultiArray>::SharedPtr contact_schedule_sub_;
    rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<QuadrupedEKF> ekf_;

    // MuJoCo variables
    mjModel* m_ptr_;
    mjData* d_ptr_;

    // Member variables from your source file that were missing from your header
    JointState last_joint_data_;
    Imu last_imu_data_; // <-- Add a variable to store the last IMU data
    std::vector<bool> contact_schedule_;
    bool is_initialized_ = false;

    // Mapping from joint names to MuJoCo qpos/qvel indices
    std::map<std::string, int> joint_to_qpos_index_map_;

    //  viewer
    std::thread viewer_thread_;
    bool run_viewer_ = false;
    void viewer_loop();    
    
};