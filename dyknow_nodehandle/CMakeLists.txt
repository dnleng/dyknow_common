cmake_minimum_required(VERSION 2.8.3)
project(dyknow_nodehandle)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED cmake_modules COMPONENTS roscpp std_msgs geometry_msgs message_filters topic_tools nodelet message_generation dyknow_analytics)

add_message_files(
    FILES
    Configuration.msg
    Field.msg
    Sample.msg
)

add_service_files(
    FILES
    Configure.srv
    GetConfig.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs message_filters topic_tools nodelet dyknow_analytics
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
    ${dyknow_analytics_INCLUDE_DIRS}
)

add_library(dyknow_nodehandle
    src/node_handle.cpp
    src/subscriber.cpp
    src/publisher.cpp
    src/proxy.cpp
    src/timer.cpp
)

add_dependencies(dyknow_nodehandle
    dyknow_nodehandle_gencpp
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dyknow_nodehandle
    ${catkin_LIBRARIES}
    ${dyknow_nodehandle_LIBRARIES}
)
