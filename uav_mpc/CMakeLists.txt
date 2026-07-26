# SKELETON — the structure is complete, the acados/target wiring marked TODO is not.
# See IMPLEMENTATION_GUIDE.md §3.
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
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# TODO(deepseek): locate acados.
#   - honour ${ACADOS_SOURCE_DIR} from the environment; error out with an actionable message
#     if UAV_MPC_WITH_ACADOS is ON and it is unset
#   - ACADOS_INCLUDE_DIRS = ${ACADOS_SOURCE_DIR}/include, .../include/blasfeo/include,
#     .../include/hpipm/include
#   - the generated tree lives in ${CMAKE_CURRENT_SOURCE_DIR}/../codegen/codegen_output/c_generated_code
#   - import libacados.so, libblasfeo.so, libhpipm.so as IMPORTED targets
#   - add a custom command that re-runs codegen/generate_acados_solver.py when
#     codegen/quadrotor_model.py or config/nmpc_params.yaml is newer than the generated code

# ------------------------------------------------------------------------------------------------
# Interfaces
# ------------------------------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SolverDiagnostics.msg"
  "msg/TrajectorySpec.msg"
  "msg/NmpcStatus.msg"
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
# TODO(deepseek): when UAV_MPC_WITH_ACADOS, add the acados include dirs + link the
# generated solver library, and target_compile_definitions(... PRIVATE UAV_MPC_WITH_ACADOS).

# ------------------------------------------------------------------------------------------------
# Node (component + executable)
# ------------------------------------------------------------------------------------------------
add_library(nmpc_component SHARED src/nmpc_node.cpp)
target_link_libraries(nmpc_component ${PROJECT_NAME}_core)
ament_target_dependencies(nmpc_component
  rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs
  std_msgs geometry_msgs nav_msgs px4_msgs tf2 tf2_ros tf2_eigen Eigen3
)
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
install(TARGETS ${PROJECT_NAME}_core nmpc_component
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

  ament_add_gtest(test_trajectory_continuity test/test_trajectory_continuity.cpp)
  target_link_libraries(test_trajectory_continuity ${PROJECT_NAME}_core)

  # TODO(deepseek): add test/test_dynamics.cpp (finite-difference Jacobian check) and
  # test/test_frame_conversions.cpp (ENU<->NED round-trip) once the core is implemented.

  ament_add_pytest_test(test_acados_codegen test/test_acados_codegen.py
    TIMEOUT 600
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core nmpc_component)
ament_export_dependencies(rclcpp rclcpp_lifecycle px4_msgs Eigen3 rosidl_default_runtime)
ament_package()
