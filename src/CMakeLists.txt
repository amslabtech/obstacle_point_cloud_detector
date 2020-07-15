add_library(obstacle_point_cloud_detector SHARED
    obstacle_point_cloud_detector_component.cpp
)

target_compile_definitions(obstacle_point_cloud_detector PRIVATE "OBSTACLE_POINTCLOUD_DETECTOR_BUILDING_DLL")

target_link_libraries(obstacle_point_cloud_detector
    ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_point_cloud_detector
    rclcpp
    rclcpp_components
    sensor_msgs
)

add_executable(obstacle_point_cloud_detector_node
    obstacle_point_cloud_detector_node.cpp
)

target_link_libraries(obstacle_point_cloud_detector_node
    obstacle_point_cloud_detector
    ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_point_cloud_detector_node
    rclcpp
    rclcpp_components
    sensor_msgs
)

install(TARGETS
    obstacle_point_cloud_detector_node
    DESTINATION lib/obstacle_point_cloud_detector
)

install(TARGETS
    obstacle_point_cloud_detector
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_libraries(obstacle_point_cloud_detector)
