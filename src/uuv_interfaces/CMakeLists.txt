cmake_minimum_required(VERSION 3.8)
project(uuv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add action files
set(action_files
  "action/PoseToPose.action"
)

# Add service files
set(srv_files
  "srv/GenerateHelix.srv"
  "srv/BayesianSearch.srv"
  "srv/DubinsLengths.srv"
  "srv/GenerateCone.srv"
)

# Add message files
set(msg_files
  "msg/FaceHits.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
