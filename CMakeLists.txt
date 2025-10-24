cmake_minimum_required(VERSION 3.5)
project(kf_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------------------
# Find dependent packages
# --------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(odrive_can REQUIRED)
find_package(Eigen3 REQUIRED) # <-- Keep this line

find_package(osc_2_in_interface REQUIRED)

find_package(glfw3 REQUIRED)
# --------------------------------------------------------------------------------------
# --- MuJoCo Integration ---
# --------------------------------------------------------------------------------------
set(MUJOCO_PATH "/home/vivek/.mujoco/mujoco-3.2.7") # Update with your MuJoCo path
include_directories(${MUJOCO_PATH}/include)
include_directories(${MUJOCO_PATH}/include/mujoco)
link_directories(${MUJOCO_PATH}/lib)

# --- Eigen Integration ---
# set(EIGEN_PATH "/usr/include/eigen3") # Common path for Eigen3
# include_directories(${EIGEN_PATH})
# --------------------------------------------------------------------------------------
# --- EKF Executable (Updated Name) ----------------------------------------------------------------
# --------------------------------------------------------------------------------------
add_executable(ekf_estimator_node
  src/ekf_estimator_node.cpp
)
target_include_directories(ekf_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS} # <-- Use the variable set by find_package  
)
target_link_libraries(ekf_estimator_node mujoco Eigen3::Eigen glfw)
ament_target_dependencies(ekf_estimator_node rclcpp nav_msgs sensor_msgs std_msgs)
install(TARGETS ekf_estimator_node DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------------------------------
# --- Contact Scheduler Executable -----------------------------------------------------------------
# --------------------------------------------------------------------------------------
add_executable(contact_scheduler_node
  src/contact_scheduler_node.cpp
)
# target_link_libraries(contact_scheduler_node
#   rclcpp::rclcpp
#   std_msgs::std_msgs
# )
ament_target_dependencies(contact_scheduler_node rclcpp std_msgs)
install(TARGETS contact_scheduler_node DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------------------------------
# --- ODrive Adapter Executable -------------------------------------------------------------------
# --------------------------------------------------------------------------------------
add_executable(odrive_adapter_node
  src/odrive_adapter_node.cpp
)
target_include_directories(odrive_adapter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# target_link_libraries(odrive_adapter_node
#   rclcpp::rclcpp
#   sensor_msgs::sensor_msgs
#   odrive_can::odrive_can
# )
ament_target_dependencies(odrive_adapter_node rclcpp sensor_msgs odrive_can)
install(TARGETS odrive_adapter_node DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------------------------------
# --- State Converter Executable (NEW NODE) --------------------------------------------
# --------------------------------------------------------------------------------------
add_executable(state_converter_node
  src/state_converter_node.cpp
)
target_include_directories(state_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${osc_2_in_interface_INCLUDE_DIRS} # Include path for custom message headers
)
target_link_libraries(state_converter_node 
  ${osc_2_in_interface_LIBRARIES} # Link against the custom message library
)
ament_target_dependencies(state_converter_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  osc_2_in_interface # Dependency for ament_export
)
install(TARGETS state_converter_node DESTINATION lib/${PROJECT_NAME})

ament_package()