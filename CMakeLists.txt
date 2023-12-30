cmake_minimum_required(VERSION 3.5)
project(loam_back_and_forth)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(tf2 REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Qhull REQUIRED)
find_package(FLANN REQUIRED)
find_package(geometry2 REQUIRED)



include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
   ${PCL_INCLUDE_DIRS} 
   #/opt/ros/humble/include/tf2_geometry_msgs/tf2_geometry_msgs.h
   ${pigpio_INCLUDE_DIR} 
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  tf2_geometry_msgs
  geometry2
)


add_executable(scanRegistration_bf src/scanRegistration.cpp)
#target_include_directories(scanRegistration_bf PUBLIC ${CMAKE_SOURCE_DIR}/geometry2-rolling/tf2 )
ament_target_dependencies(scanRegistration_bf ${dependencies} )
target_link_libraries(scanRegistration_bf ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(laserOdometry_bf src/laserOdometry.cpp)
ament_target_dependencies(laserOdometry_bf ${dependencies})
target_link_libraries(laserOdometry_bf ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(laserMapping_bf src/laserMapping.cpp)
ament_target_dependencies(laserMapping_bf ${dependencies})
target_link_libraries(laserMapping_bf ${PCL_LIBRARIES} ${OpenCV_LIBS} )

add_executable(transformMaintenance_bf src/transformMaintenance.cpp)
ament_target_dependencies(transformMaintenance_bf ${dependencies})
target_link_libraries(transformMaintenance_bf ${PCL_LIBRARIES} ${OpenCV_LIBS} )



install(TARGETS
  scanRegistration_bf
  laserOdometry_bf
  laserMapping_bf
  transformMaintenance_bf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()


