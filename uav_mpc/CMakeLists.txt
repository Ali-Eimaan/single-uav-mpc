# Build system for the uav_mpc package. See .deepseek/03_BUILD_SYSTEM.md §3.
cmake_minimum_required(VERSION 3.16)
project(uav_mpc LANGUAGES C CXX)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
# Release must be -O2 or better; the solve-time test assumes an optimised build.
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

option(UAV_MPC_WITH_ACADOS
  "Link the acados-generated solver. OFF builds a stub backend for lint/unit CI." ON)

# ------------------------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# px4_msgs is an OPTIONAL EXTRA, never a dependency. It is not released for ROS 2 Lyrical Luth,
# so the whole package — node included — builds and runs without it using the generic vehicle
# backend (standard ROS 2 messages). When px4_msgs IS present, the PX4 backend is compiled in
# additionally and can be selected at runtime with `vehicle_interface:=px4`.
#
# To enable the PX4 backend:
#   git clone https://github.com/PX4/px4_msgs.git <ros2_ws>/src/px4_msgs
#   cd <ros2_ws> && colcon build
# CMake picks it up automatically on the next configure. Force it off with
# -DUAV_MPC_WITH_PX4_MSGS=OFF even when the package is installed.
option(UAV_MPC_WITH_PX4_MSGS
  "Compile the PX4 (px4_msgs/uXRCE-DDS) vehicle backend when px4_msgs is available." ON)

set(UAV_MPC_PX4_BACKEND OFF)
if(UAV_MPC_WITH_PX4_MSGS)
  find_package(px4_msgs QUIET)
  if(px4_msgs_FOUND)
    set(UAV_MPC_PX4_BACKEND ON)
    message(STATUS
      "px4_msgs found — compiling the PX4 vehicle backend (select with vehicle_interface:=px4)")
  else()
    message(STATUS
      "px4_msgs NOT found — building with the generic vehicle backend only. This is expected "
      "on Lyrical Luth; the node still builds and runs. Install px4_msgs to enable "
      "vehicle_interface:=px4.")
  endif()
else()
  message(STATUS "UAV_MPC_WITH_PX4_MSGS=OFF — PX4 backend deliberately excluded")
endif()
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# ------------------------------------------------------------------------------------------------
# acados (§3.1) and the generated solver
# ------------------------------------------------------------------------------------------------
set(CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../codegen)
set(GENERATED_SOLVER_DIR ${CODEGEN_DIR}/codegen_output/c_generated_code)

if(UAV_MPC_WITH_ACADOS)
  if(NOT DEFINED ENV{ACADOS_SOURCE_DIR})
    message(FATAL_ERROR
      "UAV_MPC_WITH_ACADOS is ON but ACADOS_SOURCE_DIR is not set. Install acados from "
      "source and export ACADOS_SOURCE_DIR (see .deepseek/02_ENVIRONMENT.md). "
      "To build the stub backend instead, configure with -DUAV_MPC_WITH_ACADOS=OFF.")
  endif()
  set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})
  set(ACADOS_INCLUDE_DIRS
    ${ACADOS_SOURCE_DIR}/include
    ${ACADOS_SOURCE_DIR}/include/blasfeo/include
    ${ACADOS_SOURCE_DIR}/include/hpipm/include
  )

  # Import the prebuilt acados runtime libraries.
  add_library(libacados SHARED IMPORTED)
  set_target_properties(libacados PROPERTIES IMPORTED_LOCATION ${ACADOS_SOURCE_DIR}/lib/libacados.so)
  add_library(libblasfeo SHARED IMPORTED)
  set_target_properties(libblasfeo PROPERTIES IMPORTED_LOCATION ${ACADOS_SOURCE_DIR}/lib/libblasfeo.so)
  add_library(libhpipm SHARED IMPORTED)
  set_target_properties(libhpipm PROPERTIES IMPORTED_LOCATION ${ACADOS_SOURCE_DIR}/lib/libhpipm.so)

  # The generated solver library (codegen/generate_acados_solver.py).
  add_library(libacados_ocp_solver_quadrotor SHARED IMPORTED)
  set_target_properties(libacados_ocp_solver_quadrotor PROPERTIES
    IMPORTED_LOCATION ${GENERATED_SOLVER_DIR}/libacados_ocp_solver_quadrotor.so)

  # Regenerate whenever the model or the structural config changes. Skipped in CI: the
  # check-only job re-runs codegen to a temp dir and diffs (see .deepseek/14_CI.md §14.1).
  if(NOT DEFINED ENV{CI})
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_custom_command(
      OUTPUT ${GENERATED_SOLVER_DIR}/acados_solver_quadrotor.c
      COMMAND ${Python3_EXECUTABLE} generate_acados_solver.py
      WORKING_DIRECTORY ${CODEGEN_DIR}
      DEPENDS
        ${CODEGEN_DIR}/quadrotor_model.py
        ${CODEGEN_DIR}/generate_acados_solver.py
        ${CMAKE_CURRENT_SOURCE_DIR}/config/nmpc_params.yaml
      VERBATIM
      COMMENT "Regenerating acados solver (codegen/generate_acados_solver.py)"
    )
    add_custom_target(uav_mpc_codegen DEPENDS ${GENERATED_SOLVER_DIR}/acados_solver_quadrotor.c)
  endif()
endif()

# ------------------------------------------------------------------------------------------------
# Interfaces
# ------------------------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SolverDiagnostics.msg"
  "msg/TrajectorySpec.msg"
  "msg/NmpcStatus.msg"
  "msg/AttitudeThrustSetpoint.msg"
  "srv/SetTrajectory.srv"
  "action/FollowTrajectory.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# ------------------------------------------------------------------------------------------------
# Core library
# ------------------------------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core SHARED
  src/quadrotor_dynamics.cpp
  src/trajectory_generator.cpp
  src/acados_wrapper.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core Eigen3 yaml_cpp_vendor)
if(UAV_MPC_WITH_ACADOS)
  target_include_directories(${PROJECT_NAME}_core PRIVATE
    ${ACADOS_INCLUDE_DIRS}
    ${GENERATED_SOLVER_DIR}
    ${CODEGEN_DIR}/codegen_output/include   # model_hash.h
  )
  # PUBLIC so test_nmpc_solve_time can skip when the stub backend is built.
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC UAV_MPC_WITH_ACADOS)
  target_link_libraries(${PROJECT_NAME}_core
    libacados libblasfeo libhpipm libacados_ocp_solver_quadrotor)
  if(TARGET uav_mpc_codegen)
    add_dependencies(${PROJECT_NAME}_core uav_mpc_codegen)
  endif()
endif()
# acados_wrapper.cpp reads codegen/MODEL_HASH for the stale-solver guard (§6.7).
target_compile_definitions(${PROJECT_NAME}_core PRIVATE
  UAV_MPC_CODEGEN_DIR="${CODEGEN_DIR}"
)

# ------------------------------------------------------------------------------------------------
# Node (component + executable) — ALWAYS built. px4_msgs is not required.
# ------------------------------------------------------------------------------------------------
# vehicle_interface.cpp holds the generic backend + the factory and always compiles.
# vehicle_interface_px4.cpp is guarded by UAV_MPC_WITH_PX4_MSGS internally, so it is safe to
# list unconditionally — without the define it compiles to an empty translation unit.
add_library(nmpc_component SHARED
  src/nmpc_node.cpp
  src/vehicle_interface.cpp
  src/vehicle_interface_px4.cpp
)
target_link_libraries(nmpc_component ${PROJECT_NAME}_core)
ament_target_dependencies(nmpc_component
  rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs
  std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_eigen Eigen3
)
if(UAV_MPC_PX4_BACKEND)
  target_compile_definitions(nmpc_component PRIVATE UAV_MPC_WITH_PX4_MSGS)
  ament_target_dependencies(nmpc_component px4_msgs)
endif()
rclcpp_components_register_node(nmpc_component
  PLUGIN "uav_mpc::NmpcNode"
  EXECUTABLE nmpc_node_component
)
# Link the node against this package's own generated interfaces.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(nmpc_component "${cpp_typesupport_target}")

add_executable(nmpc_node src/main.cpp)
target_link_libraries(nmpc_node nmpc_component ${PROJECT_NAME}_core)
ament_target_dependencies(nmpc_node rclcpp rclcpp_lifecycle)

# ------------------------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nmpc_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS nmpc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config params rviz DESTINATION share/${PROJECT_NAME})

# ------------------------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # copyright/license headers are hand-managed; everything else runs.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_nmpc_solve_time test/test_nmpc_solve_time.cpp)
  target_link_libraries(test_nmpc_solve_time ${PROJECT_NAME}_core)
  ament_target_dependencies(test_nmpc_solve_time Eigen3 yaml_cpp_vendor)
  target_compile_definitions(test_nmpc_solve_time PRIVATE
    UAV_MPC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
    UAV_MPC_CODEGEN_DIR="${CODEGEN_DIR}"
  )

  ament_add_gtest(test_trajectory_continuity test/test_trajectory_continuity.cpp)
  target_link_libraries(test_trajectory_continuity ${PROJECT_NAME}_core)
  target_compile_definitions(test_trajectory_continuity PRIVATE
    UAV_MPC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

  # M1: finite-difference Jacobian check + ENU<->NED round-trips (§10.4).
  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_link_libraries(test_dynamics ${PROJECT_NAME}_core)
  target_compile_definitions(test_dynamics PRIVATE
    UAV_MPC_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")

  ament_add_gtest(test_frame_conversions test/test_frame_conversions.cpp)
  target_link_libraries(test_frame_conversions ${PROJECT_NAME}_core)

  # Helper binary used by test_acados_codegen.py to compare CasADi against C++ (§10.3).
  add_executable(dynamics_probe test/dynamics_probe.cpp)
  target_link_libraries(dynamics_probe ${PROJECT_NAME}_core)
  target_compile_definitions(dynamics_probe PRIVATE
    UAV_MPC_DEFAULT_AIRFRAME_PARAMS="${CMAKE_CURRENT_SOURCE_DIR}/params/x500_calibration.yaml")

  ament_add_pytest_test(test_acados_codegen test/test_acados_codegen.py
    TIMEOUT 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core nmpc_component)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle geometry_msgs nav_msgs Eigen3 rosidl_default_runtime)
# px4_msgs is deliberately NOT exported even when present: it is an internal implementation
# detail of one backend, and exporting it would push the dependency onto every consumer.
ament_package()
