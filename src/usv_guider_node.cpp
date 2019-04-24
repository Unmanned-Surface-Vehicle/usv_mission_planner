#include "ros/ros.h"                                  // ROS
#include "std_msgs/String.h"                          // ROS_INFO
#include <nav_msgs/Odometry.h>                        // usv_guider command msg

#include "../include/usv_guider/GridWithWeights.hpp"  // A*
#include "../include/usv_guider/PriorityQueue.hpp"    // A*
#include "../include/usv_guider/AStar.hpp"            // A*

#include <sstream>

#define ROS_MAIN_LOOP___TIME_INTERVAL                         0.2
#define PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND  10
#define SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION       1
#define ACCEPTABLE_RADIUS                                     2

std::queue<Pos>* A_Star();
void usv_state_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg);

geometry_msgs::Point usv_current_pos;

int main(int argc, char **argv){
  
  // Basic configuration to start node and configure a publisher
  ros::init(argc, argv, "usv_guider_node");                                                                                                         // Basic iniitalization for ROS node
  ros::NodeHandle n;                                                                                                                                // Handler to interact with ROS
  ros::Publisher  usv_guider_pub  = n.advertise<nav_msgs::Odometry>("/airboat/move_usv/goal", PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND);// Declaration of ROS topic and creation of a publishing handler for usv_guider waypoint command 
  ros::Subscriber usv_guider_sub  = n.subscribe("/airboat/state", SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION, usv_state_callback);
  ros::Rate loop_rate(ROS_MAIN_LOOP___TIME_INTERVAL);                                                                                               // Runs CA strategy each 1/PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND secs.  

  // USV Guider position command message
  nav_msgs::Odometry usv_guider_command_position_msg;
  usv_guider_command_position_msg.header.stamp = ros::Time::now();
  usv_guider_command_position_msg.header.frame_id  = "world";  

  // Gets A* output path
  std::queue<Pos> *path = A_Star();

  // Gets first subgoal location
  Pos next_goal;
  next_goal.x = path->front().x;
  next_goal.y = path->front().y;
  path->pop();

  // Handler to publish usv subgoal location
  geometry_msgs::Point point;
  point.x = next_goal.x;
  point.y = next_goal.y;
  point.z = 0;

  // publish first subgoal location
  usv_guider_command_position_msg.pose.pose.position = point;
  usv_guider_pub.publish(usv_guider_command_position_msg);

  while (ros::ok()){

    ROS_INFO("Current x: %f y: %f",   usv_current_pos.x, usv_current_pos.y);
    ROS_INFO("Next    x: %f y: %f\n", next_goal.x, next_goal.y);

    // Set new subgoal location after reach current subgoal
    if((abs(usv_current_pos.x - next_goal.x) <= ACCEPTABLE_RADIUS) && (abs(usv_current_pos.y - next_goal.y) <= ACCEPTABLE_RADIUS)){ // Tests if achieved subgoal

      if(!path->empty()){

        next_goal = path->front();                                  // Gets next subgoal location
        path->pop();                                                // Removes from path data structure

        // Update subgoal
        point.x = next_goal.x;
        point.y = next_goal.y;
        usv_guider_command_position_msg.pose.pose.position = point;

      }else{

        delete path;

      }

    }

    usv_guider_pub.publish(usv_guider_command_position_msg);

    // Attending to subscription callbacks
    ros::spinOnce();
    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}


void usv_state_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

  usv_current_pos = usv_position_msg->pose.pose.position;
  // ROS_INFO("USV current position: X: %f, Y: %f", usv_current_pos.x, usv_current_pos.y);
  // ROS_INFO("C_x - N_x: %f - %f --- C_y - N_y: %f - %f", usv_current_pos.x, next_goal.x, usv_current_pos.y, next_goal.y);

}

std::queue<Pos>* A_Star(){

  std::queue<Pos> *path = new std::queue<Pos>();
  Pos test;

  test.x = 240;
  test.y = 95;
  path->push(test);

  test.x = 260;
  test.y = 95;
  path->push(test);

  test.x = 240;
  test.y = 95;
  path->push(test);

  test.x = 260;
  test.y = 95;
  path->push(test);

  test.x = 240;
  test.y = 95;
  path->push(test);

  test.x = 260;
  test.y = 95;
  path->push(test);
  

  return path;
  
}