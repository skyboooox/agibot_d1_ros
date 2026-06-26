cmake_minimum_required(VERSION 3.16)
project(agibot_d1_ros VERSION 0.1.0 LANGUAGES CXX)

option(AGIBOT_D1_ENABLE_SDK "Build the live Agibot D1 SDK bridge node" ON)
set(
  AGIBOT_D1_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/agibot_d1_sdk"
  CACHE PATH
  "Path to the official agibot_d1_sdk checkout"
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(agibot_d1_policy INTERFACE)
target_include_directories(agibot_d1_policy INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(policy_self_test test/policy_self_test.cpp)
  target_link_libraries(policy_self_test agibot_d1_policy)
  add_test(NAME policy_self_test COMMAND policy_self_test)
endif()

if(AGIBOT_D1_ENABLE_SDK)
  if(NOT AGIBOT_D1_SDK_ROOT)
    message(FATAL_ERROR "AGIBOT_D1_SDK_ROOT is required when AGIBOT_D1_ENABLE_SDK=ON")
  endif()

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(AGIBOT_D1_ARCH "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(AGIBOT_D1_ARCH "aarch64")
  else()
    message(FATAL_ERROR "Unsupported architecture for Agibot D1 SDK: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  set(AGIBOT_D1_SDK_INCLUDE "${AGIBOT_D1_SDK_ROOT}/include")
  set(AGIBOT_D1_SDK_LIB_DIR "${AGIBOT_D1_SDK_ROOT}/lib/zsl-1/${AGIBOT_D1_ARCH}")
  set(AGIBOT_D1_SDK_LIB "${AGIBOT_D1_SDK_LIB_DIR}/libmc_sdk_zsl_1_${AGIBOT_D1_ARCH}.so")

  if(NOT EXISTS "${AGIBOT_D1_SDK_INCLUDE}/zsl-1/highlevel.h")
    message(FATAL_ERROR "Missing zsl-1/highlevel.h under ${AGIBOT_D1_SDK_INCLUDE}")
  endif()
  if(NOT EXISTS "${AGIBOT_D1_SDK_LIB}")
    message(FATAL_ERROR "Missing Agibot D1 SDK library: ${AGIBOT_D1_SDK_LIB}")
  endif()

  add_library(agibot_d1_sdk SHARED IMPORTED GLOBAL)
  set_target_properties(agibot_d1_sdk PROPERTIES
    IMPORTED_LOCATION "${AGIBOT_D1_SDK_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${AGIBOT_D1_SDK_INCLUDE}"
  )

  add_executable(d1_highlevel_bridge_node src/d1_highlevel_bridge_node.cpp)
  target_link_libraries(d1_highlevel_bridge_node
    agibot_d1_policy
    agibot_d1_sdk
  )
  ament_target_dependencies(d1_highlevel_bridge_node
    geometry_msgs
    rclcpp
    std_msgs
  )
  target_compile_definitions(d1_highlevel_bridge_node PRIVATE AGIBOT_D1_ENABLE_SDK=1)

  add_executable(d1_state_node src/d1_state_node.cpp)
  target_link_libraries(d1_state_node agibot_d1_sdk)
  ament_target_dependencies(d1_state_node
    rclcpp
    std_msgs
  )
  set_target_properties(d1_highlevel_bridge_node d1_state_node PROPERTIES
    BUILD_RPATH "${AGIBOT_D1_SDK_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN"
  )

  install(TARGETS
    d1_highlevel_bridge_node
    d1_state_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(FILES "${AGIBOT_D1_SDK_LIB}" DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "AGIBOT_D1_ENABLE_SDK=OFF: d1_highlevel_bridge_node will not be built")
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS
  setup.sh
  setup_local.sh
  setup_default.sh
  scripts/d1_live_smoke
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/d1_live_smoke DESTINATION lib/${PROJECT_NAME})

ament_package()
