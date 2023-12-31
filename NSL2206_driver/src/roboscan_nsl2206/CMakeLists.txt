cmake_minimum_required(VERSION 3.8)
project(roboscan_nsl2206)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(camera_info_manager REQUIRED COMPONENTS)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(cv_bridge)

find_package(image_transport)
find_package(nav_msgs)
find_package(std_msgs)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


link_directories(${PCL_LIBRARY_DIRS})
#add_definition($PCL_DEFINITIONS})

include_directories(include/ 
${PCL_INCLUDE_DIRS} 
${Boost_INCLUDE_DIRS} 
${OpenCV_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS})

include_directories(
  include
  include/communication  
  ${ament_cmake_INCLUDE_DIRS}
)



# Build
add_executable(roboscan_publish_node
                src/roboscan_publish_node.cpp
                #src/camera2206_driver.cpp
                src/communication/blocking_command_helper.cpp
                src/communication/chip_information_helper.cpp
                src/communication/communication.cpp
                src/communication/communication_2206.cpp
                src/communication/crc_calc.cpp
                src/communication/production_information_helper.cpp
                src/communication/serial_connection.cpp
                src/communication/u32_helper.cpp
                src/communication/u16_helper.cpp
                src/communication/update_controller.cpp
                src/communication/util.cpp
                src/communication/HdrHandler.cpp
                src/communication/roboscan_nsl2206_image.cpp
                src/communication/roboscan_nsl2206_grayscale_image.cpp
                src/communication/roboscan_nsl2206_distance_image.cpp
                src/communication/roboscan_nsl2206_distance_amplitude_image.cpp
                src/communication/roboscan_nsl2206_distance_grayscale_image.cpp
                src/communication/camera_calibration
               )
                                
target_include_directories(roboscan_publish_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(roboscan_publish_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(roboscan_publish_node
  sensor_msgs
  rclcpp
  std_msgs
  cv_bridge
  pcl_conversions
  Eigen3
  OpenCV
)

target_link_libraries(roboscan_publish_node 
${PCL_LIBRARIES}
${Boost_SYSTEM_LIBRARY}
${Boost_LIBRARIES})

install(TARGETS roboscan_publish_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
set(CMAKE_BUILD_TYPE Release)

ament_export_include_directories(include) 
ament_package()
