#include <motion_controller/motion_controller.h>

  /**
   * TODO
   *  - Invert reverse?
   **/

namespace motion_controller
{
  MotionController::MotionController() 
  : loopFreq_(50)
  , vLinMax_(0.0)
  , vAngMax_(0.0)
  {
    ros::NodeHandle nh, private_nh("~");
    ros::Rate loopRate(loopFreq_);

    //load and calculate parameters
    private_nh.param<double>("max_rpm", wheelRpmMax_, 3000);
    private_nh.param<double>("min_rpm", wheelRpmMin_, 300);
    private_nh.param<double>("wheel_separation", wheelSep_, 1.57);
    private_nh.param<double>("wheel_radius", wheelRadius_, 0.4191);
    private_nh.param<double>("gearbox_ratio", gearboxRatio_, 62.5);
    private_nh.param<bool>("limit_velocities", limitInputVels_, false);
    private_nh.param<double>("max_linear_velocity", vLinMax_, 1.5);
    private_nh.param<double>("max_angular_velocity", vAngMax_, 0.9);
    private_nh.param<bool>("prevent_wheel_direction_change", preventWDirectionChange_, false);
    private_nh.param<bool>("prevent_tiny_inner_wheel_speed", preventTinyInnerWheelSpeed_, false);
    private_nh.param<int>("canbus_nodeid_left", canbusNodeIdLeft_, 0x26);
    private_nh.param<int>("canbus_nodeid_right", canbusNodeIdRight_, 0x27);
    

    if( limitInputVels_ && (!vLinMax_ || !vAngMax_) )
    {
      ROS_WARN("Limiter is enabled but linear and/or angular velocities limits parameters were not set. "
        "Input velocities won't be adjusted.");
      limitInputVels_ = false;
    }

    distConstant_ = 2 * M_PI * wheelRadius_ / (60 * gearboxRatio_);
    wheelVelMax_ =  wheelRpmMax_ * distConstant_;
    wheelVelMin_ = wheelRpmMin_ * distConstant_;
    ROS_INFO("wheel velocity limit (m/s): Max=%.2f, Min=%.2f", wheelVelMax_, wheelVelMin_);

    odomMsg_.header.frame_id = "odom";
    odomMsg_.child_frame_id = "base_link";

    // ROS publishers and subscribers
    cmdVel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MotionController::cmdVelCB, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);

    // Other objects
    curtisLeft_ = new curtis_interface::CurtisInterface(canbusNodeIdLeft_);
    curtisRight_ = new curtis_interface::CurtisInterface(canbusNodeIdRight_);

    // Encoder data callback
    readCurtis_tm_ = nh.createTimer(ros::Duration(1.0/loopFreq_), &MotionController::readCurtisTimerCB, this);
    
  }

  MotionController::~MotionController()
  {
    delete curtisLeft_;
    delete curtisRight_;
  }

  void MotionController::cmdVelCB(const geometry_msgs::Twist::ConstPtr &msg)
  {
    double vLeft, vRight;
    cmdVelToWheelVel(msg->linear.x, msg->angular.z, vLeft, vRight);
    
    curtisLeft_->setThrottle( vLeft / wheelVelMax_);
    //add delay??
    curtisLeft_->setThrottle( vRight / wheelVelMax_);
  }
  
  void MotionController::readCurtisTimerCB(const ros::TimerEvent&)
  {
    double vLeft = curtisLeft_->getRPM() * distConstant_;
    double vRight = curtisRight_->getRPM() * distConstant_;
    double odomVLin = (vRight + vLeft) / 2.0;
    double odomVAng = (vRight - vLeft) / wheelSep_;

    // publish odometry message
    odomMsg_.header.stamp = ros::Time::now();
    odomMsg_.twist.twist.linear.x = odomVLin;
    odomMsg_.twist.twist.angular.z = odomVAng;
    odom_pub_.publish(odomMsg_);
  }

  void MotionController::cmdVelToWheelVel(double vLin, double vAng, double &vLeft, double &vRight)
  {
    /**
     * Main formulas
     *  - vAng = (vLeft - vRight) / ws  
     *  - vLin = (vLeft + vRight) / 2.0
     * 
     * Design choices
     *  1 - robot should not be allowed to translate with either left or right velocities equal to zero
     *  2 - while steering the velocity of the inner wheel must be above wheelVelMin_. Velocity may be capped.
     **/

    // shared variables
    short int vLinDirection = (0.0 < vLin) - (vLin < 0.0);
    short int vAngDirection = (0.0 < vAng) - (vAng < 0.0);
    double coe = 0.0;

    //limit input velocities.
    if(limitInputVels_)
    {
      if(fabs(vLin) > vLinMax_)
      {
        ROS_WARN("Linear velocity inputted exceeds the pre-defined maximum velocity (max_linear_velocity). "
          "Using the maximum allowed value instead.");
        vLin = vLinMax_ * vLinDirection;
      }
      if(fabs(vAng) > vAngMax_)
      {
        ROS_WARN("Angular velocity inputted exceeds the pre-defined maximum velocity (max_angular_velocity). "
          "Using the maximum allowed value instead.");
        vAng = vAngMax_ * vAngDirection;
      }
    }

    // initial wheel velocities calculation
    vLeft = vLin - vAng * wheelSep_ / 2.0;
    vRight = vLin + vAng * wheelSep_ / 2.0;

    //correct velocities in steering scenarios
    if(fabs(vLin) > 0.0 && fabs(vAng) > 0.0)
    {
      // check if input would change the direction of a wheel
      // This adjustment distort the intended trajectory arc!
      if(preventWDirectionChange_)
      {
        if( fabs(vLin) < fabs(vAng) * wheelSep_ / 2.0)
        {
          ROS_WARN("Invalid command. Intended input pair of velocities would change the direction of a wheel. "
            "Limiting angular velocity!");
          if(vLinDirection * vAngDirection == 1)
            vLeft = wheelVelMin_ * vLinDirection;
          else
            vRight = wheelVelMin_ * vLinDirection;
        }
      }

      //adjust vels if the outer wheel is above threshold
      double fastest = std::max(fabs(vLeft), fabs(vRight));
      if(fastest > wheelVelMax_)
      {
        coe = wheelVelMax_ / fastest;
        vLeft *= coe;
        vRight *= coe;
      }

      if(preventTinyInnerWheelSpeed_)
      {
        //adjust vels if the inner wheel is below threshold
        double slowest = std::min(fabs(vLeft), fabs(vRight));
        coe = wheelVelMin_ / slowest;
        // cap the velocity of the inner wheel to the minimum if its vel is too small
        if(coe > 1.5)
        {
          if(fabs(vLeft) < fabs(vRight))
            vLeft = wheelVelMin_ * vLinDirection;
          else
            vRight = wheelVelMin_ * vLinDirection;
        }
        // proportionally increase the speed of both wheels to ensure that the inner wheel is above the minimum
        else if(coe > 1.0)
        {
          vLeft *= coe;
          vRight *= coe;
        }
      }

      // (extra) cap maximum velocity if the outer velocity is still above threshold
      fastest = std::max(fabs(vLeft), fabs(vRight));
      if(fastest > wheelVelMax_)
      {
        if(fabs(vLeft) > fabs(vRight))
          vLeft = wheelVelMax_ * vLinDirection;
        else
          vRight = wheelVelMax_ * vLinDirection;
      }
    }
    // Skid + Forward/Reverse: cap velocities if they exceed wheelVelMax_;
    else
    {
      coe = wheelVelMax_ / fabs(vLeft);
      if(coe < 1 )
        vLeft *= coe;
      coe = wheelVelMax_ / fabs(vRight);
      if(coe < 1 )
        vRight *= coe;
    }
  }

} // namespace motion_controller