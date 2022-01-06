cmake_minimum_required(VERSION 3.8)
project(mynteye_wrapper_d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(mynteyed REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

# targets
add_executable(mynteye_wrapper_d_node src/mynteye_wrapper_node.cc)
target_include_directories(mynteye_wrapper_d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mynteye_wrapper_d_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(mynteye_wrapper_d_node 
  mynteyed
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  rclcpp
  image_transport
  cv_bridge
  OpenCV
  visualization_msgs
)

add_executable(mynteye_listener_d src/mynteye_listener.cc)
target_include_directories(mynteye_listener_d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mynteye_listener_d PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(mynteye_listener_d 
mynteyed
sensor_msgs
std_msgs
tf2
tf2_ros
rclcpp
image_transport
cv_bridge
OpenCV
visualization_msgs
)

#generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetParams.srv"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_target_interfaces(mynteye_wrapper_d_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(mynteye_listener_d
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install

install(TARGETS mynteye_wrapper_d_node mynteye_listener_d
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()