cmake_minimum_required(VERSION 2.8.3)
project(i3dr_pcl_tools)

find_package(catkin REQUIRED cv_bridge image_transport roscpp tf pcl_ros serial std_msgs sensor_msgs)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

add_service_files(
    FILES
    reset_map.srv
    save_map.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs sensor_msgs message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(
    pcl_reduce src/pcl_reduce.cpp
)
add_executable(
    pcl_to_mesh src/pcl_to_mesh.cpp
)
add_executable(
    pcl_map src/pcl_map.cpp
)
add_executable(
    transform_pcl_frame src/transform_pcl_frame.cpp
)
add_executable(
    join_pcl src/join_pcl.cpp
)
add_executable(
    crop_pcl src/crop_pcl.cpp
)
add_executable(
    save_pcl src/save_pcl.cpp
)
add_executable(
    save_pcl_tf_list src/save_pcl_tf_list.cpp
)
add_executable(
    load_pcl src/load_pcl.cpp
)
add_executable(
    load_pcl_tf_list src/load_pcl_tf_list.cpp
)

target_link_libraries(pcl_reduce
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(pcl_to_mesh
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(pcl_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(transform_pcl_frame
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(join_pcl
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(crop_pcl
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(save_pcl
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(save_pcl_tf_list
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(load_pcl
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

target_link_libraries(load_pcl_tf_list
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_dependencies(pcl_map ${PROJECT_NAME}_generate_messages_cpp)