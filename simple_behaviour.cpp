//http://jbohren.com/articles/roscpp-hello-world/
//https://pixhawk.org/dev/offboard_control/testing
//https://pixhawk.org/dev/ros/mavros_offboard
//http://answers.ros.org/question/202864/trouble-with-using-roscpp-and-mavros-api-to-integrate-pixhawk-with-jetson/
//http://answers.ros.org/question/122031/moving-a-simple-object/
//http://docs.opencv.org/2.4/modules/core/doc/basic_structures.html
//http://wiki.ros.org/mavros#Plugins
//http://wiki.ros.org/mavros#mavros.2BAC8-Plugins.setpoint_position
//http://wiki.ros.org/geometry_msgs

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"

ros::Publisher mavros_local_pos_pub;
ros::Publisher mavros_velocity_pub;
ros::Subscriber mavros_local_pos_sub;

geometry_msgs::PoseStamped poseMsg;
geometry_msgs::TwistStamped velocityMsg;

void goForward(){
  velocityMsg.header.stamp = ros::Time::now();
  velocityMsg.twist.linear.x = 0; 
  velocityMsg.twist.linear.y = 2; 
  velocityMsg.twist.linear.z = 0;   
  velocityMsg.twist.angular.x = 0; 
  velocityMsg.twist.angular.y = 0; 
  velocityMsg.twist.angular.z = 0;

  mavros_velocity_pub.publish(velocityMsg);
}

void stop(){
  velocityMsg.header.stamp = ros::Time::now();
  velocityMsg.twist.linear.x = 0; 
  velocityMsg.twist.linear.y = 0; 
  velocityMsg.twist.linear.z = 0;   
  velocityMsg.twist.angular.x = 0; 
  velocityMsg.twist.angular.y = 0; 
  velocityMsg.twist.angular.z = 0;

  mavros_velocity_pub.publish(velocityMsg);
}

void takeOff(){
  poseMsg.header.stamp = ros::Time::now();
  poseMsg.pose.position.x = 0; 
  poseMsg.pose.position.y = 0; 
  poseMsg.pose.position.z = 2;   
  poseMsg.pose.orientation.w = 0; 
  poseMsg.pose.orientation.x = 0; 
  poseMsg.pose.orientation.y = 0; 
  poseMsg.pose.orientation.z = 0;

  mavros_local_pos_pub.publish(poseMsg);
}

void land(){
  poseMsg.header.stamp = ros::Time::now();
  poseMsg.pose.position.x = 0; 
  poseMsg.pose.position.y = 0; 
  poseMsg.pose.position.z = 0;   
  poseMsg.pose.orientation.w = 0; 
  poseMsg.pose.orientation.x = 0; 
  poseMsg.pose.orientation.y = 0; 
  poseMsg.pose.orientation.z = 0;

  mavros_local_pos_pub.publish(poseMsg);
}

void localPosCallback(const geometry_msgs::PoseStamped & msg)
{
  if(msg.pose.position.z < 2){
    mavros_local_pos_pub.publish(poseMsg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;


  //OWN PUBLISHERS
  mavros_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint/local_position",1000);
  mavros_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint/cmd_vel",1000);
  
  //OWN SUBSCRIBES
  //mavros_local_pos_sub = nh.subscribe("~local_position/pose", 1000, localPosCallback);
  
  //ORIGINAL : For requesting Offboard control
  ros::Publisher mavros_control_pub = nh.advertise<mavros::ControlSetpoint>("/mavros/setpoint_control/setpoint",1000);
  mavros::ControlSetpoint fmu_controller_setpoint;

  ros::ServiceClient mavros_set_mode_client = nh.serviceClient<mavros::SetMode>("mavros/set_mode");
  mavros::SetMode set_mode;
  set_mode.request.custom_mode = "OFFBOARD";
  
  ros::ServiceClient mavros_nav_guided_client = nh.serviceClient<mavros::CommandBool>("/mavros/cmd/guided_enable");
  mavros::CommandBool nav_guided;
  nav_guided.request.value = true;
  
  bool offboard_commands_enabled = false;
  bool nav_guided_enabled = false;
  
  ros::Rate loop_rate(100.0);

  while(ros::ok())
  {

    // Section 1: getting offboard control
    if (!offboard_commands_enabled) {
      if (mavros_set_mode_client.call(set_mode))
      {
        ROS_INFO("Set mode: OFFBOARD enabled!");
	       offboard_commands_enabled = true;
      }
      else
      {
        ROS_INFO("Offboard mode still not enabled!");
      }
    }

    // Write desired setpoint value to fmu_controller_setpoint variable. from the demo
    mavros_control_pub.publish(fmu_controller_setpoint);


    if(!nav_guided_enabled)
    {
      if (mavros_nav_guided_client.call(nav_guided))
      {
	       nav_guided_enabled = true;
	       ROS_INFO("Nav guided: OFFBOARD enabled!");
      }
    }

    //Section 2: sending messages

    //Take Off criteria

    if(poseMsg.pose.position.z < 2){
      mavros_local_pos_pub.publish(poseMsg);
    }

    //Stop and go criteria
    bool closeToObject = true;
    
    if(closeToObject){
      stop();
      land();
    }else{
      goForward();
    }
    
    ros::spinOnce();

    loop_rate.sleep();
  }

}
