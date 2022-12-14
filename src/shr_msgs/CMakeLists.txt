cmake_minimum_required(VERSION 3.5)
project(shr_msgs)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
        action/RecognizeRequest.action
        action/RecognizeTrainRequest.action
        action/RotateRequest.action
        )

rosidl_generate_interfaces(shr_msgs
        ${msg_files}
        ${action_files}
        DEPENDENCIES
        action_msgs
        std_msgs
        builtin_interfaces
        )


ament_export_dependencies(rosidl_default_runtime)

ament_package()
