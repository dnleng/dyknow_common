cmake_minimum_required(VERSION 2.8.3)
project(dyknow_analytics)

find_package(catkin REQUIRED cmake_modules COMPONENTS roscpp std_msgs geometry_msgs message_filters topic_tools nodelet message_generation)

add_message_files(
    FILES
    Monitor.msg
    MonitorPair.msg
    MonitorSet.msg
)

#add_service_files(
#    FILES
#    Configure.srv
#    GetConfig.srv
#)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs message_filters topic_tools nodelet
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)

add_library(dyknow_analytics
    src/monitor.cpp
    src/analysis.cpp
)

add_dependencies(dyknow_analytics
    dyknow_analytics_gencpp
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dyknow_analytics
    ${catkin_LIBRARIES}
    ${dyknow_analytics_LIBRARIES}
)
