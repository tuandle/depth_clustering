add_library(ros_bridge SHARED cloud_odom_ros_subscriber.cpp cloud_ros_publisher.cpp)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
)
target_link_libraries(ros_bridge
                      identifiable
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})