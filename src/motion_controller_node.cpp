#include "motion_controller.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_controller_node");

  motion_controller::MotionController motion_controller;
  
  ros::spin();

  return(0);
}