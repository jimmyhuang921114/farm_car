cmake_minimum_required(VERSION 3.16)
project(obs_filiter)

# ------------------ 編譯選項 ------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-O3 -march=native)

# ------------------ 套件依賴 ------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)
find_package(message_filters REQUIRED)

# ------------------ include 路徑 ------------------
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ------------------ 節點一：realsense_read_node ------------------
add_executable(realsense_read_node src/realsense_read.cpp)
target_link_libraries(realsense_read_node ${OpenCV_LIBS} realsense2)
ament_target_dependencies(realsense_read_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
)

# ------------------ 節點二：plane_node（使用 message_filters） ------------------
add_executable(plane_node src/plane.cpp)
target_link_libraries(plane_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  realsense2
  OpenMP::OpenMP_CXX
  TBB::tbb
)
ament_target_dependencies(plane_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
  image_transport
  message_filters
)

# ------------------ 節點三：obstacle_node ------------------
add_executable(obstacle_node src/obstacle_detector.cpp)
target_link_libraries(obstacle_node ${PCL_LIBRARIES})
ament_target_dependencies(obstacle_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
  cv_bridge
)

# ------------------ 節點四：xz_visualizer_node ------------------
add_executable(xz_visualizer_node src/xz_visualizer_node.cpp)
target_link_libraries(xz_visualizer_node ${OpenCV_LIBS})
ament_target_dependencies(xz_visualizer_node
  rclcpp
  sensor_msgs
  cv_bridge
  pcl_conversions
)

# ------------------ 安裝 ------------------
install(TARGETS
  realsense_read_node
  plane_node
  obstacle_node
  xz_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()