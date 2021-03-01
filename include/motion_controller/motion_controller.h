#ifndef MOTION_CONTROLLER__MOTION_CONTROLLER_H
#define MOTION_CONTROLLER__MOTION_CONTROLLER_H

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "curtis_interface/curtis_interface.h"

namespace motion_controller
{

  class MotionController
  {
    public:
      MotionController();
      virtual ~MotionController();

    private:
      void cmdVelCB(const geometry_msgs::Twist::ConstPtr& msg);
      void readCurtisTimerCB(const ros::TimerEvent&);
      void cmdVelToWheelVel(double vLin, double vAng, double& vLeft, double& vRight);

      double loopFreq_;

      int canbusNodeIdLeft_;
      int canbusNodeIdRight_;

      double distConstant_;
      double wheelRpmMax_;
      double wheelRpmMin_;
      double gearboxRatio_;
      double wheelSep_;  
      double wheelRadius_;
      double wheelVelMin_;
      double wheelVelMax_;
      double vLinMax_;
      double vAngMax_;
      bool limitInputVels_;
      bool preventWDirectionChange_;
      bool preventTinyInnerWheelSpeed_;

      ros::Timer readCurtis_tm_;
      nav_msgs::Odometry odomMsg_;
      ros::Subscriber cmdVel_sub_;
      ros::Publisher odom_pub_;

      curtis_interface::CurtisInterface* curtisLeft_;
      curtis_interface::CurtisInterface* curtisRight_;

  };
}
#endif // MOTION_CONTROLLER__MOTION_CONTROLLER_H