cmake_minimum_required(VERSION 2.8.3)
project(task_handle)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        cmake_modules
        geometry_msgs
        message_generation
        move_base_msgs
        roscpp
        rospy
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        move_base_msgs
        std_msgs
        roscpp
        INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# move_base
add_library(task_handle
  src/task_handle.cpp
)
target_link_libraries(task_handle
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

install(TARGETS
    task_handle
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
#add_dependencies(task_handle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_handle_node src/task_handle_node.cpp)
#add_dependencies(task_handle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_handle_node task_handle)
