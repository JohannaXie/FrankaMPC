cmake_minimum_required(VERSION 3.4)

project(libfranka-examples CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/../cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Poco REQUIRED COMPONENTS Foundation)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(examples_common STATIC
  examples_common.cpp
)


target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3)

set(EXAMPLES
  cartesian_impedance_control
  communication_test
  echo_robot_state
  force_control
  generate_cartesian_pose_motion
  generate_cartesian_velocity_motion
  generate_consecutive_motions
  generate_elbow_motion
  generate_joint_position_motion
  generate_joint_velocity_motion
  grasp_object
  joint_impedance_control
  joint_point_to_point_motion
  motion_with_control
  print_joint_poses
  0_generate_joint_position_motion
  0_generate_joint_velocity_motion
  0_freemove_velocity
  0_freemove_torque
)

set(OBJS
  ${CMAKE_CURRENT_LIST_DIR}/solver.o
  ${CMAKE_CURRENT_LIST_DIR}/ldl.o
  ${CMAKE_CURRENT_LIST_DIR}/util.o
  ${CMAKE_CURRENT_LIST_DIR}/matrix_support.o
)

add_executable(mpc_target ${OBJS} mpc_target.cpp)
add_executable(MPC ${OBJS} MPC.cpp)
add_executable(MPC_openloop ${OBJS} MPC_openloop.cpp)
add_executable(MPC_velocity_openloop ${OBJS} MPC_velocity_openloop.cpp)
add_executable(0_MPC_velocity_openloop ${OBJS} 0_MPC_velocity_openloop.cpp)
add_executable(1_MPC_velocity_openloop ${OBJS} 1_MPC_velocity_openloop.cpp)
add_executable(0_MPC ${OBJS} 0_MPC.cpp)
target_link_libraries(mpc_target Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(MPC Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(MPC_openloop Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(MPC_velocity_openloop Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(0_MPC_velocity_openloop Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(1_MPC_velocity_openloop Franka::Franka examples_common Eigen3::Eigen3)
target_link_libraries(0_MPC Franka::Franka examples_common Eigen3::Eigen3)

 set_source_files_properties(
   ${OBJS}
   PROPERTIES
   EXTERNAL_OBJECT true
   GENERATED true
 )


foreach(example ${EXAMPLES})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} Franka::Franka examples_common Eigen3::Eigen3)
endforeach()

target_link_libraries(joint_impedance_control Threads::Threads)
target_link_libraries(motion_with_control Poco::Foundation)

include(GNUInstallDirs)
install(TARGETS ${EXAMPLES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS mpc_target
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS MPC
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS MPC_openloop
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS MPC_velocity_openloop
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS 0_MPC_velocity_openloop
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS 1_MPC_velocity_openloop
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(TARGETS 0_MPC
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
