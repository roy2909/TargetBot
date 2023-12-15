cmake_minimum_required(VERSION 3.8)
project(trajectory_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files

)

set(srv_files
  "srv/Target.srv"
  "srv/TargetScanRequest.srv"
  "srv/Grab.srv"
  "srv/UserInput.srv"
)

 rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()