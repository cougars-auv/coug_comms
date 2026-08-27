cmake_minimum_required(VERSION 3.22)
project(coug_comms)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

set(COUG_COMMS_NODES
  agent_receiver
  agent_status_bundler
  agent_status_stager
  base_dispatcher
  base_status_extractor
  base_status_poller
)

set(COUG_COMMS_NODE_SOURCES "")
set(COUG_COMMS_PARAM_LIBRARIES "")
foreach(node IN LISTS COUG_COMMS_NODES)
  generate_parameter_library(
    ${node}_parameters
    "params/${node}_params.yaml"
  )
  list(APPEND COUG_COMMS_NODE_SOURCES "src/${node}.cpp")
  list(APPEND COUG_COMMS_PARAM_LIBRARIES "${node}_parameters")
endforeach()

ament_auto_add_library(coug_comms_component SHARED NO_TARGET_LINK_LIBRARIES
  ${COUG_COMMS_NODE_SOURCES}
)

target_compile_features(coug_comms_component PUBLIC cxx_std_17)

target_link_libraries(coug_comms_component
  ${COUG_COMMS_PARAM_LIBRARIES}
  Eigen3::Eigen
)

rclcpp_components_register_nodes(coug_comms_component
  "coug_comms::AgentReceiverNode"
  "coug_comms::AgentStatusBundlerNode"
  "coug_comms::AgentStatusStagerNode"
  "coug_comms::BaseDispatcherNode"
  "coug_comms::BaseStatusExtractorNode"
  "coug_comms::BaseStatusPollerNode"
)

foreach(node IN LISTS COUG_COMMS_NODES)
  ament_auto_add_executable(${node} NO_TARGET_LINK_LIBRARIES "src/${node}_node.cpp")
  target_link_libraries(${node} coug_comms_component)
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_status_codec test/test_status_codec.cpp)
  target_include_directories(test_status_codec PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
  )
  target_link_libraries(test_status_codec
    Eigen3::Eigen
    coug_interfaces::coug_interfaces__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
