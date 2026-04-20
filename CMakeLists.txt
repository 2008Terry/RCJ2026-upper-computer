cmake_minimum_required(VERSION 3.8)
project(rcj_localization)

add_link_options("-fuse-ld=mold")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenMP REQUIRED)
# Crucially, find the OpenCV package that cv_bridge depends on
find_package(OpenCV REQUIRED)
include_directories(include)

# Vision processor reusable library
add_library(vision_processor
  src/vision_processor.cpp
)

target_link_libraries(vision_processor
  ${OpenCV_LIBRARIES}
)

add_library(white_line_debug_utils
  src/white_line_debug_utils.cpp
)

target_link_libraries(white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_library(white_line_preprocessing
  src/white_line_preprocessing.cpp
)

ament_target_dependencies(white_line_preprocessing
  rclcpp
)

target_link_libraries(white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

# --- Add Your Executable ---
# This tells cmake to build an executable file named "localization_node"
# from your source file "src/main_node.cpp"
add_executable(localization_node 
  src/main_node.cpp
  src/particle_filter.cpp
)

# --- Link Libraries ---
# This is the most important part. It connects your code to the ROS 2 and OpenCV libraries.
ament_target_dependencies(localization_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros"
  "cv_bridge"
  "visualization_msgs"
)

target_link_libraries(localization_node 
  vision_processor
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# Standalone vision debug node
add_executable(vision_processor_node
  src/vision_processor_node.cpp
)

ament_target_dependencies(vision_processor_node
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "cv_bridge"
  "visualization_msgs"
)

target_link_libraries(vision_processor_node
  vision_processor
  ${OpenCV_LIBRARIES}
)

add_executable(fastmap_remap_node
  src/ros2_fastmap_remap_node.cpp
)

ament_target_dependencies(fastmap_remap_node
  "rclcpp"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

target_link_libraries(fastmap_remap_node
  ${OpenCV_LIBRARIES}
)


# --- Installation Rules ---
# This tells colcon where to put the files so `ros2 run` can find them.
install(TARGETS
  localization_node
  vision_processor_node
  fastmap_remap_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install Python Scripts ---
install(PROGRAMS
  "scripts/yaw_publisher.py"
  "src/image_publisher.py"
  DESTINATION lib/${PROJECT_NAME}
)

# --- HSV Tuner ---
add_executable(hsv_tuner src/hsv_tuner_node.cpp)
ament_target_dependencies(hsv_tuner
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(hsv_tuner
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_edge_node src/white_line_edge_node.cpp)
ament_target_dependencies(white_line_edge_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_edge_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_preprocess_node src/white_line_preprocess_node.cpp)
ament_target_dependencies(white_line_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_preprocess_node
  white_line_preprocessing
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_adaptive_threshold_node src/white_line_adaptive_threshold_node.cpp)
ament_target_dependencies(white_line_adaptive_threshold_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_adaptive_threshold_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_cluster_node src/white_line_cluster_node.cpp)
ament_target_dependencies(white_line_cluster_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_cluster_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_hsv_cluster_node src/white_line_hsv_cluster_node.cpp)
ament_target_dependencies(white_line_hsv_cluster_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_hsv_cluster_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_hsv_white_node src/white_line_hsv_white_node.cpp)
ament_target_dependencies(white_line_hsv_white_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_hsv_white_node
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_lab_cluster_node src/white_line_lab_cluster_node.cpp)
ament_target_dependencies(white_line_lab_cluster_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_lab_cluster_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_lab_morph_node src/white_line_lab_morph_node.cpp)
ament_target_dependencies(white_line_lab_morph_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_lab_morph_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_skeleton_filter_node src/white_line_skeleton_filter_node.cpp)
ament_target_dependencies(white_line_skeleton_filter_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)
target_link_libraries(white_line_skeleton_filter_node
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(white_line_dt_ridge_filter_node src/white_line_dt_ridge_filter_node.cpp)
ament_target_dependencies(white_line_dt_ridge_filter_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)
target_link_libraries(white_line_dt_ridge_filter_node
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(topdown_mask_points_node src/topdown_mask_points_node.cpp)
ament_target_dependencies(topdown_mask_points_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
)
target_link_libraries(topdown_mask_points_node
  ${OpenCV_LIBRARIES}
)

add_executable(pf_localization_node
  src/pf_localization_node.cpp
  src/particle_filter.cpp
)
ament_target_dependencies(pf_localization_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)
target_link_libraries(pf_localization_node
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(topdown_pf_localization_node
  src/topdown_pf_localization_node.cpp
  src/particle_filter.cpp
)
ament_target_dependencies(topdown_pf_localization_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  cv_bridge
)
target_link_libraries(topdown_pf_localization_node
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(topdown_pf_localization_node_v2
  src/topdown_pf_localization_node_v2.cpp
  src/particle_filter_v2.cpp
)
ament_target_dependencies(topdown_pf_localization_node_v2
  rclcpp
  rcl_interfaces
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  cv_bridge
)
target_link_libraries(topdown_pf_localization_node_v2
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(white_line_probability_node src/white_line_probability_node.cpp)
ament_target_dependencies(white_line_probability_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(white_line_probability_node
  white_line_preprocessing
  white_line_debug_utils
  ${OpenCV_LIBRARIES}
)

add_executable(black_feature_detector_node src/black_feature_detector_node.cpp)
ament_target_dependencies(black_feature_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(black_feature_detector_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  hsv_tuner
  white_line_edge_node
  white_line_preprocess_node
  white_line_adaptive_threshold_node
  white_line_cluster_node
  white_line_hsv_cluster_node
  white_line_hsv_white_node
  white_line_lab_cluster_node
  white_line_lab_morph_node
  white_line_skeleton_filter_node
  white_line_dt_ridge_filter_node
  topdown_mask_points_node
  pf_localization_node
  topdown_pf_localization_node
  topdown_pf_localization_node_v2
  white_line_probability_node
  black_feature_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch maps config docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
