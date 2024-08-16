cmake_minimum_required(VERSION 3.10)
project(rm2_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
set(dependencies
        tf2
        tf2_msgs
        tf2_ros
        rclcpp
        geometry_msgs
        Eigen3
        rm2_msgs
        sensor_msgs
        Eigen3
        realtime_tools
        control_msgs
        imu_complementary_filter
        message_filters
        rclcpp
        controller_manager_msgs
#        imu_filter_madgwick
#        dynamic_reconfigure
)

foreach(Dependency IN ITEMS ${dependencies})
        find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
        include
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE sources "src/*.cpp" "src/decision/*.cpp" "src/filter/*.cpp")

add_library(rm2_common SHARED ${sources})
#add_executable(test_traj test/test_traj.cpp)
add_executable(test_kalman test/test_kalman_filter.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
#target_link_libraries(test_traj rm2_common ${catkin_LIBRARIES})
target_link_libraries(test_kalman rm2_common)

# Fix rm2_msgs generation problem
# See https://answers.ros.org/question/73048
#ament_target_dependencies(rm2_common rm2_msgs_generate_messages_cpp)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation
#install(
#        DIRECTORY doc
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

ament_export_libraries(rm2_common)

ament_export_dependencies(${dependencies})

#############
## Testing ##
#############

if (${CATKIN_ENABLE_TESTING})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(lqr_test
            test/unit_test_lqr.cpp
            test/LqrTest.cpp)
    target_link_libraries(lqr_test rm2_common
)
endif ()

ament_package()