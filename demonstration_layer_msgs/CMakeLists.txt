cmake_minimum_required(VERSION 2.8.3)
project(demonstration_layer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(FILES
  BucketWeight.msg
  FeatureWeight.msg
  MacroCellWeight.msg
  Weights.msg
)

generate_messages()

catkin_package()
