cmake_minimum_required(VERSION 2.8.3)
project(yocs_vm_interactor)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vending_machine_msgs
)

catkin_python_setup()
catkin_package()
