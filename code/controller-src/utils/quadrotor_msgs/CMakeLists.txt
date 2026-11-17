cmake_minimum_required(VERSION 3.10)
project(quadrotor_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs)
add_message_files(FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  LQRTrajectory.msg
  Odometry.msg
  OutputData.msg
  PolynomialTrajectory.msg
  PositionCommand.msg
  PPROutputData.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg)
add_service_files(FILES SetTakeoffLand.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs)
