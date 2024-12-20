/*  This node republishes cmd vel with stamped twist instead of simple twist from cmd_vel */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/TwistStamped.h>

#include <sstream>          



ros::Publisher *pubCmdVelHusky;      // pointer to publisher object
geometry_msgs::TwistStamped cmdVelHuskyOut;



void cmdVelHusky0_cbk(const geometry_msgs::Twist::ConstPtr& cmdVelHuskyIn){
  cmdVelHuskyOut.twist.linear = cmdVelHuskyIn->linear;
  cmdVelHuskyOut.twist.angular = cmdVelHuskyIn->angular;
  cmdVelHuskyOut.header.stamp = ros::Time();

  pubCmdVelHusky->publish(cmdVelHuskyOut);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rep_cmd_vel_husky_stamped");
  ros::NodeHandle n;

  ros::Subscriber sub0 = n.subscribe("/husky_velocity_controller/cmd_vel", 100, &cmdVelHusky0_cbk);
  pubCmdVelHusky = new ros::Publisher(  n.advertise<geometry_msgs::TwistStamped>("/husky_velocity_controller/cmd_vel_stamped", 1000)  );  

  std::cout << " \t Republishing stamped /husky_velocity_controller/cmd_vel " << std:: endl;
  ros::spin();
  delete pubCmdVelHusky;
}



