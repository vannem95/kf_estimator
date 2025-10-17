#include "kf_estimator/ekf_estimator_node.hpp"
#include "stdio.h"
#include "string.h"
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>


// ----------------------------- viewer -----------------------------
// Add these MuJoCo and GLFW headers
#include <mujoco/mjrender.h>
#include <GLFW/glfw3.h>
#include <chrono>

mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window;

// Define the viewer callback function
void EKFNode::viewer_loop() {
    // initialize GLFW and create window
    if (!glfwInit()) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize GLFW.");
        return;
    }
    window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize MuJoCo rendering
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(m_ptr_, &scn, 2000);
    mjr_makeContext(m_ptr_, &con, mjFONTSCALE_100);
    
    cam.distance = 2.0;
    cam.azimuth = 90;
    cam.elevation = -45;

    run_viewer_ = true;

    while (!glfwWindowShouldClose(window) && run_viewer_) {
        // Update the MuJoCo scene with the latest data
        mjv_updateScene(m_ptr_, d_ptr_, &opt, NULL, &cam, mjCAT_ALL, &scn);

        // Render the scene
        mjrRect rect = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &rect.width, &rect.height);
        mjr_render(rect, &scn, &con);

        // Swap buffers and poll for events
        glfwSwapBuffers(window);
        glfwPollEvents();
        
        // Sleep to limit frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Clean up
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwTerminate();
}
// ----------------------------- viewer -----------------------------



// --- QuadrupedEKF Class Implementations ---
QuadrupedEKF::QuadrupedEKF(double dt, mjModel* model, mjData* data) :
    dt_(dt), model_ptr_(model), data_ptr_(data)
{
    // Initialize state and covariance matrices
    x_ = VectorXd::Zero(N_TOTAL_STATES);
    P_ = MatrixXd::Identity(N_TOTAL_STATES, N_TOTAL_STATES);
    Q_ = MatrixXd::Identity(N_TOTAL_STATES, N_TOTAL_STATES) * 1e-4; // Process noise
    R_ = MatrixXd::Identity(NUM_FEET * 3 + N_JOINT_POS + N_JOINT_VEL, NUM_FEET * 3 + N_JOINT_POS + N_JOINT_VEL) * 1e-2; // Measurement noise    
}




void QuadrupedEKF::initialize_state(const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& joint_vel) {
    // Set base position to zero
    x_.segment(0, N_BASE_POS).setZero();
    // Set base orientation to identity quaternion (no rotation)
    x_(3) = 1.0;
    x_(4) = 0.0;
    x_(5) = 0.0;
    x_(6) = 0.0;
    // Set base velocities to zero
    x_.segment(7, N_BASE_LIN_VEL).setZero();
    x_.segment(10, N_BASE_ANG_VEL).setZero();
    
    // Initialize joint positions (starts at index 13)
    x_.segment(13, N_JOINT_POS) = joint_pos;
    // Initialize joint velocities (starts at index 13 + N_JOINT_POS = 21)
    x_.segment(21, N_JOINT_VEL) = joint_vel;
}

void QuadrupedEKF::predict(const Vector3d& base_acc, const Vector3d& base_gyro) {
    // 1. Unpack the current state vector x_
    Vector3d base_pos = x_.head(3);
    Quaterniond base_orient_quat(x_(3), x_(4), x_(5), x_(6));
    Vector3d base_lin_vel = x_.segment(7, 3);
    Vector3d base_ang_vel = x_.segment(10, 3);
    VectorXd joint_pos = x_.segment(13, N_JOINT_POS);
    VectorXd joint_vel = x_.segment(13 + N_JOINT_POS, N_JOINT_VEL);

    // 2. Propagate the state using a constant velocity model for joints and a rigid body model for the base
    
    // a) Propagate base state using IMU measurements
    // Compensate for gravity
    // Vector3d g_world(0.0, 0.0, 9.81);
    // Vector3d g_body = base_orient_quat.inverse() * g_world;
    // Vector3d base_acc_compensated = base_acc - g_body;

    // Convert compensated IMU measurements to the world frame using the current orientation estimate
    // Vector3d world_lin_acc = base_orient_quat * base_acc_compensated; 
    Vector3d world_lin_acc = base_orient_quat * base_acc; 
    
    // Predict new position: P_k+1 = P_k + v_k * dt + 0.5 * a_k * dt^2
    Vector3d new_base_pos = base_pos + base_lin_vel * dt_ + 0.5 * world_lin_acc * dt_ * dt_;
    
    // Predict new linear velocity: v_k+1 = v_k + a_k * dt
    Vector3d new_base_lin_vel = base_lin_vel + world_lin_acc * dt_;
    
    // Propagate base orientation using manual quaternion integration
    double dw = -0.5 * (base_orient_quat.x() * base_gyro.x() + base_orient_quat.y() * base_gyro.y() + base_orient_quat.z() * base_gyro.z()) * dt_;
    double dx = 0.5 * (base_orient_quat.w() * base_gyro.x() + base_orient_quat.y() * base_gyro.z() - base_orient_quat.z() * base_gyro.y()) * dt_;
    double dy = 0.5 * (base_orient_quat.w() * base_gyro.y() - base_orient_quat.x() * base_gyro.z() + base_orient_quat.z() * base_gyro.x()) * dt_;
    double dz = 0.5 * (base_orient_quat.w() * base_gyro.z() + base_orient_quat.x() * base_gyro.y() - base_orient_quat.y() * base_gyro.x()) * dt_;

    Quaterniond new_base_orient_quat;
    new_base_orient_quat.w() = base_orient_quat.w() + dw;
    new_base_orient_quat.x() = base_orient_quat.x() + dx;
    new_base_orient_quat.y() = base_orient_quat.y() + dy;
    new_base_orient_quat.z() = base_orient_quat.z() + dz;
    new_base_orient_quat.normalize();

    // b) Propagate joint states using a constant velocity model
    VectorXd new_joint_pos = joint_pos + joint_vel * dt_;
    VectorXd new_joint_vel = joint_vel; // Velocity remains constant in prediction step

    // 3. Update the state vector x_ with predicted values
    x_.head(3) = new_base_pos;
    x_.segment(3, 4) << new_base_orient_quat.w(), new_base_orient_quat.x(), new_base_orient_quat.y(), new_base_orient_quat.z();
    x_.segment(7, 3) = new_base_lin_vel;
    x_.segment(10, 3) = base_ang_vel; 
    x_.segment(13, N_JOINT_POS) = new_joint_pos;
    x_.segment(13 + N_JOINT_POS, N_JOINT_VEL) = new_joint_vel;

    // setting z to zero till imu is calibrated
    x_(2) = 0.0;

    // 4. Compute the Jacobian of the state transition function F
    MatrixXd F = MatrixXd::Identity(N_TOTAL_STATES, N_TOTAL_STATES);
    F.block(0, 7, 3, 3) = MatrixXd::Identity(3, 3) * dt_;
    F.block(13, 13 + N_JOINT_POS, N_JOINT_POS, N_JOINT_VEL) = MatrixXd::Identity(N_JOINT_POS, N_JOINT_VEL) * dt_;

    // Update the process noise matrix (Q) to include IMU noise.
    MatrixXd Q_accel(3, 3);
    Q_accel.setIdentity();
    Q_.block(7, 7, 3, 3) = Q_accel * 9.62e-5; // For linear velocity (from acceleration noise)

    MatrixXd Q_gyro(3, 3);
    Q_gyro.setIdentity();
    Q_.block(10, 10, 3, 3) = Q_gyro * 1.49e-6; // For angular velocity (from gyroscope noise)
    // Propagate covariance
    P_ = F * P_ * F.transpose() + Q_;
}

void QuadrupedEKF::update(const Eigen::VectorXd& sensor_z, const std::vector<bool>& contact_states, mjModel* m_ptr, mjData* d_ptr) {

    // 0. Check EKF state for NaN or infinite values
    bool has_inf = false;
    for (int i = 0; i < x_.rows(); ++i) {
        if (std::isinf(x_(i))) {
            has_inf = true;
            break;
        }
    }
    
    if (x_.hasNaN() || has_inf) {
        std::cerr << "EKF State has NaN or Inf values! Resetting state." << std::endl;
        // Optionally, reset the state or throw an exception
        return; 
    }
    
    // Check for large values
    for (int i = 0; i < x_.rows(); ++i) {
        if (std::abs(x_(i)) > 1e6) { 
            std::cerr << "EKF State element " << i << " is extremely large: " << x_(i) << std::endl;
            // You might want to reset the filter here as well
            return;
        }
    }
    
    
    // 1. Map EKF state to MuJoCo data structure
    d_ptr->qpos[0] = x_(0);
    d_ptr->qpos[1] = x_(1);
    d_ptr->qpos[2] = x_(2);
    d_ptr->qpos[3] = x_(3);
    d_ptr->qpos[4] = x_(4);
    d_ptr->qpos[5] = x_(5);
    d_ptr->qpos[6] = x_(6);
    for (int i = 0; i < N_JOINT_POS; ++i) {
        d_ptr->qpos[7 + i] = x_(13 + i);
    }
    
    // Set MuJoCo qvel from EKF state for Jacobians
    d_ptr->qvel[0] = x_(7);
    d_ptr->qvel[1] = x_(8);
    d_ptr->qvel[2] = x_(9);
    d_ptr->qvel[3] = x_(10);
    d_ptr->qvel[4] = x_(11);
    d_ptr->qvel[5] = x_(12);

    for (int i = 0; i < N_JOINT_VEL; ++i) {
        d_ptr->qvel[6 + i] = x_(13 + N_JOINT_POS + i);
    }

    mj_fwdPosition(m_ptr, d_ptr);
    mj_fwdVelocity(m_ptr, d_ptr);

    // mj_fwdConstraint is necessary for sensor data computation
    mj_fwdConstraint(m_ptr, d_ptr);     




    // --- START DEBUGGING: PRINT FOOT VELOCITY SENSOR READINGS ---
    
    std::cout << "\n----------------- FOOT VELOCITY SENSORS (MEASUREMENTS) -----------------" << std::endl;
    
    // Assuming the order of your sensor indices matches the order of your feet (0 to 7)
    // You should iterate through the sensor indices you stored during initialization
    
    for (int i = 0; i < NUM_FEET; ++i) {
        // Get the starting address of the 3D velocity measurement (Vx, Vy, Vz)
        int sensor_data_adr = foot_vel_sensor_indices[i];
        
        // Use Eigen::Map to access the data easily
        Eigen::Map<const Vector3d> measured_foot_vel(
            &d_ptr->sensordata[sensor_data_adr]
        );

        std::cout << "Foot " << i << " velocity [Vx, Vy, Vz]: " 
                  << measured_foot_vel.transpose() 
                  << std::endl;
    }
    std::cout << "------------------------------------------------------------------------" << std::endl;

    // --- END DEBUGGING ---    






    // 2. Build dynamic measurement vector and Jacobian based on contact state
    const int NUM_FEET = 8;
    std::vector<int> active_feet_indices;
    int foot_site_indices[NUM_FEET] = {3, 4, 7, 8, 11, 12, 15, 16};
    // int foot_site_indices[NUM_FEET] = {11, 12, 15, 16, 3, 4, 7, 8};

    // Foot Site ID: 3, Name: tlf_wheel_site
    // Foot Site ID: 4, Name: tlr_wheel_site
    // Foot Site ID: 7, Name: trf_wheel_site
    // Foot Site ID: 8, Name: trr_wheel_site
    // Foot Site ID: 11, Name: hlf_wheel_site
    // Foot Site ID: 12, Name: hlr_wheel_site
    // Foot Site ID: 15, Name: hrf_wheel_site
    // Foot Site ID: 16, Name: hrr_wheel_site    

    for(size_t i = 0; i < NUM_FEET; ++i) {
        // std::cout << "--------------here--------------" << std::endl;
        // std::cout << "contact_states:" << std::endl;
        // std::cout << "contact_states:" << contact_states[i] << std::endl;
        if(contact_states[i]) {
            active_feet_indices.push_back(foot_site_indices[i]);
        }
    }
    
    
    const int num_active_feet = active_feet_indices.size();
    const int MEASUREMENT_SIZE = num_active_feet * 3 + N_JOINT_POS + N_JOINT_VEL;
    
    VectorXd z = VectorXd::Zero(MEASUREMENT_SIZE);
    z.segment(num_active_feet * 3, N_JOINT_POS + N_JOINT_VEL) = sensor_z;
    
    VectorXd z_hat = VectorXd::Zero(MEASUREMENT_SIZE);
    MatrixXd H = MatrixXd::Zero(MEASUREMENT_SIZE, N_TOTAL_STATES);
    
    // 3. Compute predicted measurements (z_hat) and Jacobian (H) for ZUPT
    for(int i = 0; i < num_active_feet; ++i) {
        int foot_idx = active_feet_indices[i];
        
        mjtNum* jac_vel_ptr = new mjtNum[3 * m_ptr->nv]; 
        mj_jacSite(m_ptr, d_ptr, NULL, jac_vel_ptr, foot_idx);

        Eigen::Map<MatrixXd> jac_vel(jac_vel_ptr, 3, m_ptr->nv);
        H.block(i*3, 0, 3, m_ptr->nv) = jac_vel;
        
        Eigen::Map<VectorXd> qvel_eigen(d_ptr->qvel, m_ptr->nv);
        z_hat.segment(i*3, 3) = jac_vel * qvel_eigen;
        
        delete[] jac_vel_ptr;
    }

    
    // 4. Compute predicted measurements and Jacobian for Joint Positions and Velocities
    z_hat.segment(num_active_feet * 3, N_JOINT_POS) = x_.segment(13, N_JOINT_POS);
    H.block(num_active_feet * 3, 13, N_JOINT_POS, N_JOINT_POS) = MatrixXd::Identity(N_JOINT_POS, N_JOINT_POS);
    
    z_hat.segment(num_active_feet * 3 + N_JOINT_POS, N_JOINT_VEL) = x_.segment(13 + N_JOINT_POS, N_JOINT_VEL);
    H.block(num_active_feet * 3 + N_JOINT_POS, 13 + N_JOINT_POS, N_JOINT_VEL, N_JOINT_VEL) = MatrixXd::Identity(N_JOINT_VEL, N_JOINT_VEL);
    

    // displaying z and z hat for debugging
    // std::cout << "--------------z--------------" << std::endl;
    // for (int element : z) {
    //     std::cout << element << "  ";
    // }
    // std::cout << std::endl;

    // std::cout << "--------------z hat--------------" << std::endl;
    // for (int element : z_hat) {
    //     std::cout << element << "  ";
    // }
    // std::cout << std::endl;    
    

    // 5. Compute Kalman Gain and update
    MatrixXd R_update = MatrixXd::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * 1e-4;
    MatrixXd S = H * P_ * H.transpose() + R_update;
    MatrixXd K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * (z - z_hat);
    P_ = (MatrixXd::Identity(N_TOTAL_STATES, N_TOTAL_STATES) - K * H) * P_;
}

// --- EKFNode Class Implementations ---
EKFNode::EKFNode() : Node("ekf_estimator_node") {

    RCLCPP_INFO(this->get_logger(), "EKF Node starting...");

    m_ptr_ = mj_loadXML("/home/vivek/projects/ros2_ws/src/kf_estimator/config/WaLTER_Senior.xml", NULL, NULL, 0);
    d_ptr_ = mj_makeData(m_ptr_);


    ekf_ = std::make_unique<QuadrupedEKF>(DT, m_ptr_, d_ptr_);


    // --- NEW: Map Sensor Names to mjData Index ---
    const int NUM_FEET = 8;
    std::string sensor_names[NUM_FEET] = {
        "tlf_wheel_vel", "tlr_wheel_vel", "trf_wheel_vel", "trr_wheel_vel",
        "hlf_wheel_vel", "hlr_wheel_vel", "hrf_wheel_vel", "hrr_wheel_vel"
    };

    for (int i = 0; i < NUM_FEET; ++i) {
        int sensor_id = mj_name2id(m_ptr_, mjOBJ_SENSOR, sensor_names[i].c_str());
        if (sensor_id >= 0) {
            ekf_->foot_vel_sensor_indices[i] = m_ptr_->sensor_adr[sensor_id];
        } else {
            RCLCPP_ERROR(this->get_logger(), "Sensor %s not found in XML!", sensor_names[i].c_str());
        }
    }    


    joint_state_sub_ = this->create_subscription<JointState>(
        "joint_states_in", 10,
        std::bind(&EKFNode::joint_state_callback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<Imu>(
        "imu/data_raw", 10,
        std::bind(&EKFNode::imu_data_callback, this, std::placeholders::_1));

    contact_schedule_sub_ = this->create_subscription<Int8MultiArray>(
        "contact_schedule", 10,
        std::bind(&EKFNode::contact_schedule_callback, this, std::placeholders::_1));

    odometry_pub_ = this->create_publisher<Odometry>("odometry/filtered", 10);
    
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(DT),
        std::bind(&EKFNode::estimator_loop, this));

    // Start the viewer thread
    viewer_thread_ = std::thread(&EKFNode::viewer_loop, this);
        
}

EKFNode::~EKFNode() {

    // Signal the viewer thread to stop
    run_viewer_ = false;
    // Wait for the thread to finish before the node shuts down
    if (viewer_thread_.joinable()) {
        viewer_thread_.join();
    }    
    
    mj_deleteData(d_ptr_);
    mj_deleteModel(m_ptr_);
}

void EKFNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    last_joint_data_ = *msg;
    
    // Build the map once at startup
    if (joint_to_qpos_index_map_.empty()) {
        for(int i = 0; i < m_ptr_->njnt; ++i) {
            std::string joint_name = mj_id2name(m_ptr_, mjOBJ_JOINT, i);
            joint_to_qpos_index_map_[joint_name] = m_ptr_->jnt_qposadr[i];
            // RCLCPP_INFO(this->get_logger(), "Mapped MuJoCo joint '%s' (ID %d) to qpos index %d", joint_name.c_str(), i, m_ptr_->jnt_qposadr[i]);            
        }
    }

    if (!is_initialized_ && last_joint_data_.position.size() == N_JOINT_POS && last_joint_data_.velocity.size() == N_JOINT_VEL) {
        
        Eigen::VectorXd initial_pos(N_JOINT_POS);
        Eigen::VectorXd initial_vel(N_JOINT_VEL);

        // Define a set of all reversed joint names
        std::set<std::string> reversed_joints = {
        "head_left_thigh_joint",     // FL-Hip
        "head_left_thigh_shin_joint",// FL-Knee
        // "torso_left_thigh_joint",    // RL-Hip
        // "torso_left_thigh_shin_joint",// RL-Knee
        "torso_right_thigh_joint",    // RL-Hip
        "torso_right_thigh_shin_joint",// RL-Knee
        };
        
        // Correctly map joint states from ROS message to Eigen vectors using the map
        std::map<int, double> temp_pos_map;
        std::map<int, double> temp_vel_map;

        for (size_t i = 0; i < last_joint_data_.name.size(); ++i) {
            std::string joint_name = last_joint_data_.name[i];
            
            if (joint_to_qpos_index_map_.count(joint_name)) {
                int qpos_index = joint_to_qpos_index_map_[joint_name];
                
                double pos_val = last_joint_data_.position[i] / 2.0;
                double vel_val = last_joint_data_.velocity[i] / 2.0;

                // Check if the current joint is in the set of reversed joints
                if (reversed_joints.count(joint_name)) {
                    pos_val *= -1.0;
                    vel_val *= -1.0;
                }

                temp_pos_map[qpos_index] = pos_val;
                temp_vel_map[qpos_index] = vel_val;

            } else {
                RCLCPP_WARN(this->get_logger(), "Received joint state for unknown joint: %s", joint_name.c_str());
            }
        }
        
        // Populate the Eigen vectors from the temporary maps, ensuring correct order
        int current_pos_idx = 0;
        int current_vel_idx = 0;
        for (auto const& [q_idx, pos_val] : temp_pos_map) {
            if (q_idx >= 7) {
                initial_pos(current_pos_idx++) = pos_val;
            }
        }
        for (auto const& [q_idx, vel_val] : temp_vel_map) {
            if (q_idx >= 7) {
                initial_vel(current_vel_idx++) = vel_val;
            }
        }

        ekf_->initialize_state(initial_pos, initial_vel);
        is_initialized_ = true;
    }
}

void EKFNode::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_data_ = *msg;
}

void EKFNode::contact_schedule_callback(const std_msgs::msg::Int8MultiArray::SharedPtr msg) {
    if (msg->data.size() == NUM_FEET) {
        contact_schedule_.clear();
        for (int8_t val : msg->data) {
            contact_schedule_.push_back(val == 1);
        }
    }
}

void EKFNode::estimator_loop() {
    if (!is_initialized_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "EKF not initialized. Waiting for joint state data.");
        return;
    }

    // Add a check to ensure contact_schedule_ has been received and populated.
    if (contact_schedule_.empty()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "EKF skipping update. Waiting for contact schedule data.");
        return;
    }    

    Vector3d base_acc(last_imu_data_.linear_acceleration.x,
                      last_imu_data_.linear_acceleration.y,
                      last_imu_data_.linear_acceleration.z);
    Vector3d base_gyro(last_imu_data_.angular_velocity.x,
                       last_imu_data_.angular_velocity.y,
                       last_imu_data_.angular_velocity.z);

    ekf_->predict(base_acc, base_gyro);

    const int N_JOINT_MEASUREMENTS = N_JOINT_POS + N_JOINT_VEL;
    VectorXd sensor_z_joints = VectorXd::Zero(N_JOINT_MEASUREMENTS);

    // if (last_joint_data_.position.size() == N_JOINT_POS && last_joint_data_.velocity.size() == N_JOINT_VEL) {
    //     sensor_z_joints.head(N_JOINT_POS) = Eigen::Map<Eigen::VectorXd>(last_joint_data_.position.data(), N_JOINT_POS);
    //     sensor_z_joints.tail(N_JOINT_VEL) = Eigen::Map<Eigen::VectorXd>(last_joint_data_.velocity.data(), N_JOINT_VEL);
    // }

    // resolving the odrive joint position is 2x real position
    if (last_joint_data_.position.size() == N_JOINT_POS && last_joint_data_.velocity.size() == N_JOINT_VEL) {
        // Correctly map joint states from ROS message to Eigen vectors using the map
        std::map<int, double> temp_pos_map;
        std::map<int, double> temp_vel_map;
        for (size_t i = 0; i < last_joint_data_.name.size(); ++i) {
            std::string joint_name = last_joint_data_.name[i];
            if (joint_to_qpos_index_map_.count(joint_name)) {
                int qpos_index = joint_to_qpos_index_map_[joint_name];
                temp_pos_map[qpos_index] = last_joint_data_.position[i] / 2.0;
                temp_vel_map[qpos_index] = last_joint_data_.velocity[i] / 2.0;
            }
        }
        
        int current_pos_idx = 0;
        int current_vel_idx = 0;
        for (auto const& [q_idx, pos_val] : temp_pos_map) {
            if (q_idx >= 7) {
                sensor_z_joints(current_pos_idx++) = pos_val;
            }
        }
        for (auto const& [q_idx, vel_val] : temp_vel_map) {
            if (q_idx >= 7) {
                sensor_z_joints(N_JOINT_POS + current_vel_idx++) = vel_val;
            }
        }
    }    
    
    ekf_->update(sensor_z_joints, contact_schedule_, m_ptr_, d_ptr_);


    // Map the final, updated EKF state back to the MuJoCo data structure for visualization
    d_ptr_->qpos[0] = ekf_->get_state()(0);
    d_ptr_->qpos[1] = ekf_->get_state()(1);
    d_ptr_->qpos[2] = ekf_->get_state()(2);
    d_ptr_->qpos[3] = ekf_->get_state()(3);
    d_ptr_->qpos[4] = ekf_->get_state()(4);
    d_ptr_->qpos[5] = ekf_->get_state()(5);
    d_ptr_->qpos[6] = ekf_->get_state()(6);
    
    for (int i = 0; i < N_JOINT_POS; ++i) {
        d_ptr_->qpos[7 + i] = ekf_->get_state()(13 + i);
    }
    for (int i = 0; i < N_JOINT_VEL; ++i) {
        d_ptr_->qvel[6 + i] = ekf_->get_state()(13 + N_JOINT_POS + i);
    }
    
    mj_fwdPosition(m_ptr_, d_ptr_);
    mj_fwdVelocity(m_ptr_, d_ptr_);

    
    Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = ekf_->get_state()(0);
    odom_msg.pose.pose.position.y = ekf_->get_state()(1);
    odom_msg.pose.pose.position.z = ekf_->get_state()(2);

    Quaterniond q(ekf_->get_state()(3), ekf_->get_state()(4), ekf_->get_state()(5), ekf_->get_state()(6));
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();

    odom_msg.twist.twist.linear.x = ekf_->get_state()(7);
    odom_msg.twist.twist.linear.y = ekf_->get_state()(8);
    odom_msg.twist.twist.linear.z = ekf_->get_state()(9);

    odom_msg.twist.twist.angular.x = ekf_->get_state()(10);
    odom_msg.twist.twist.angular.y = ekf_->get_state()(11);
    odom_msg.twist.twist.angular.z = ekf_->get_state()(12);

    odometry_pub_->publish(odom_msg);


    // std::cout << "-------------- x_ --------------" << std::endl;
    // for (int element : ekf_->get_state()) {
    //     std::cout << element << "  ";
    // }

    
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}