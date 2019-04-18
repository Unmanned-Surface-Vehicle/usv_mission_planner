#include "ros/ros.h"                                  // ROS
#include "std_msgs/String.h"                          // ROS_INFO
#include <nav_msgs/Odometry.h>                        // usv_guider command msg

#include "../include/usv_guider/GridWithWeights.hpp"  // A*
#include "../include/usv_guider/PriorityQueue.hpp"    // A*
#include "../include/usv_guider/AStar.hpp"            // A*

#include <sstream>

// std::queue<Pos>* A_Star();
void usv_state_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

  geometry_msgs::Point usv_current_pos;
  usv_current_pos = usv_position_msg->pose.pose.position;
  ROS_INFO("USV current position: X: %f, Y: %f", usv_current_pos.x, usv_current_pos.y);

}

#define PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND 0.2
#define PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND  10
#define SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION       1000

int main(int argc, char **argv){
  
  // Basic configuration to start node and configure a publisher
  ros::init(argc, argv, "usv_guider_node");                                                                                                         // Basic iniitalization for ROS node
  ros::NodeHandle n;                                                                                                                                // Handler to interact with ROS
  ros::Publisher usv_guider_pub   = n.advertise<nav_msgs::Odometry>("/airboat/move_usv/goal", PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND);  // Declaration of ROS topic and creation of a publishing handler for usv_guider waypoint command 
  ros::Subscriber usv_guider_sub  = n.subscribe("/airboat/state", SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION, usv_state_callback);
  ros::Rate loop_rate(PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND);                                                                       // Runs CA strategy each 1/PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND secs.  

  // USV Guider position command message
  nav_msgs::Odometry usv_guider_command_position_msg;
  usv_guider_command_position_msg.header.stamp = ros::Time::now();
  usv_guider_command_position_msg.header.frame_id  = "world";  

  // Target position goal for usv
  geometry_msgs::Point point;
  point.x = 255;
  point.y = 0;
  point.z = 0;
  usv_guider_command_position_msg.pose.pose.position = point;

  // std::queue<Pos> *path;
  // path = A_Star();

  // Pos next_goal = path->front();
  // path->pop();


  while (ros::ok())
  {

    // while(1){ //while(usv.currentPos() != usv_guider.getFinalGoal())

      // if(usv_current_pos.x != next_goal.x || usv_current_pos.y != next_goal.y){

      //   if(!path->empty()){

      //     next_goal = path->front();
      //     path->pop();

      //     point.x = next_goal.x;
      //     point.y = next_goal.y;
      //     usv_guider_command_position_msg.pose.pose.position = point;

      //   }

      // }

    // }
    
    ROS_INFO("Test >>>");
    usv_guider_pub.publish(usv_guider_command_position_msg);
    ros::spinOnce();
    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}

// std::queue<Pos>* A_Star(){

//   std::queue<Pos> *path;

//   path->push(Pos{255,0});
//   path->push(Pos{255,20});

//   return path;
// }