/*  This node republishes cmd vel with stamped twist instead of simple twist from cmd_vel */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/TwistStamped.h>

#include <sstream>          



ros::Publisher *pubCmdVel0;      // pointer to publisher object
geometry_msgs::TwistStamped cmdVelOut;



void cmdVel0_cbk(const geometry_msgs::Twist::ConstPtr& cmdVelIn){
  // Copy incoming cmdVelIn
  // cmdVelOut.header = cmdVelIn->header;     
  cmdVelOut.twist.linear = cmdVelIn->linear;
  cmdVelOut.twist.angular = cmdVelIn->angular;
  cmdVelOut.header.stamp = ros::Time();

  pubCmdVel0->publish(cmdVelOut);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rep_cmd_vel_stamped");
  ros::NodeHandle n;

  ros::Subscriber sub0 = n.subscribe("/cmd_vel", 100, &cmdVel0_cbk);
  pubCmdVel0 = new ros::Publisher(  n.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped", 1000)  );  //pc_out

  std::cout << " \t Republishing stamped cmd_vel " << std:: endl;
  // std::cout << " \t Republishing stamped cmd_vel and /husky_velocity_controller/cmd_vel! " << std:: endl;

  ros::spin();
  delete pubCmdVel0;
}



