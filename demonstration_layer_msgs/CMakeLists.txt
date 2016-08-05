cmake_minimum_required(VERSION 2.8.3)
project(demonstration_layer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  recovery_supervisor_msgs
  nav_msgs
)

add_message_files(FILES
  BucketWeight.msg
  FeatureWeight.msg
  MacroCellWeight.msg
  Weights.msg
)

add_service_files(FILES
  Cost.srv
)

generate_messages(DEPENDENCIES nav_msgs recovery_supervisor_msgs)

catkin_package()
