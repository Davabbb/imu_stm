cmake_minimum_required(VERSION 3.5)
project(mcu_interface) #TODO same name as in package.xml
#if(NOT CMAKE_CXX_STANDARD)
#    set(CMAKE_CXX_STANDARD 17)
#endif()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#    add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${sensor_msgs_INCLUDE_DIRS})
find_package(std_msgs REQUIRED)

find_package(serial REQUIRED)
include_directories(serial_INCLUDE_DIRS)

find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Temperature.msg

  #DEPENDENCIES
  #std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

file(GLOB SERIAL_SOURCES "./src/serial/*.cc")

add_executable(mcu src/mcu.cpp ${SERIAL_SOURCES})
target_include_directories(mcu PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

ament_target_dependencies(mcu rclcpp)
install(TARGETS
        mcu
        DESTINATION lib/${PROJECT_NAME})

target_link_libraries(mcu ${rclcpp_LIBRARIES} ${Boost_LIBRARIES} ${serial_LIBRARIES} ${sensor_msgs_LIBRARIES})

ament_package()

