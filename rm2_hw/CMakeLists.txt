cmake_minimum_required(VERSION 3.10)
project(rm2_hw)

## Use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
        rclcpp
        rm2_common
        rm2_msgs
        hardware_interface
        controller_interface
        transmission_interface
        joint_limits_interface
        controller_manager
        urdf
        realtime_tools
)

foreach(Dependency IN ITEMS ${dependencies})
        find_package(${Dependency} REQUIRED)
endforeach()

## Find system libraries
#find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
        include
)

add_library(${PROJECT_NAME}_loader_plugins
        src/transmission/multi_actuator_transmission_loader.cpp
        )
## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/${PROJECT_NAME}.cpp
        src/hardware_interface/hardware_interface.cpp
        src/hardware_interface/parse.cpp
        src/hardware_interface/socketcan.cpp
        src/hardware_interface/can_bus.cpp
        src/hardware_interface/gpio_manager.cpp
        src/control_loop.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_loader_plugins
        ${dependencies}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_loader_plugins
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )
install(
        FILES rm_hw_loader_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

#if (${CATKIN_ENABLE_TESTING})
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
#  ## Add gtest based cpp test target and link libraries
#  catkin_add_gtest(${PROJECT_NAME}-test
#      test/test_ros_package_template.cpp
#      test/AlgorithmTest.cpp)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
#endif ()

ament_export_libraries(${PROJECT_NAME}_loader_plugins)
ament_export_dependencies(${dependencies})

ament_package()