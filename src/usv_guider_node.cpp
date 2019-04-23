#include "ros/ros.h"                                  // ROS
#include "std_msgs/String.h"                          // ROS_INFO
#include <nav_msgs/Odometry.h>                        // usv_guider command msg

#include "../include/usv_guider/GridWithWeights.hpp"  // A*
#include "../include/usv_guider/PriorityQueue.hpp"    // A*
#include "../include/usv_guider/AStar.hpp"            // A*

#include <sstream>

#define ROS_MAIN_LOOP___TIME_INTERVAL                         0.2
#define PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND  10
#define SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION       1000
#define ACCEPTABLE_RADIUS                                     2

std::queue<Pos>* A_Star();
geometry_msgs::Point usv_current_pos;
Pos next_goal;

void usv_state_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

  usv_current_pos = usv_position_msg->pose.pose.position;
  ROS_INFO("USV current position: X: %f, Y: %f", usv_current_pos.x, usv_current_pos.y);
  // ROS_INFO("C_x - N_x: %f - %f --- C_y - N_y: %f - %f", usv_current_pos.x, next_goal.x, usv_current_pos.y, next_goal.y);

}

int main(int argc, char **argv){
  
  // Basic configuration to start node and configure a publisher and a subscriber
  ros::init(argc, argv, "usv_guider_node");                                                                                                         // Basic iniitalization for ROS node
  ros::NodeHandle n;                                                                                                                                // Handler to interact with ROS
  ros::Publisher  usv_guider_pub  = n.advertise<nav_msgs::Odometry>("/airboat/move_usv/goal", PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND);  // Declaration of ROS topic and creation of a publishing handler for usv_guider waypoint command 
  ros::Subscriber usv_guider_sub  = n.subscribe("/airboat/state", SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION, usv_state_callback);
  ros::Rate loop_rate(ROS_MAIN_LOOP___TIME_INTERVAL);                                                                       // Runs CA strategy each 1/ROS_MAIN_LOOP___TIME_INTERVAL secs.  

  // USV Guider position command message to be send through publisher
  nav_msgs::Odometry usv_guider_command_position_msg;
  usv_guider_command_position_msg.header.stamp    = ros::Time::now();
  usv_guider_command_position_msg.header.frame_id = "world";  

  // Gets A* output path
  std::queue<Pos> *path;
  path = A_Star();

  // Gets first subgoal location
  // Pos next_goal = path->front();
  next_goal = path->front();
  path->pop();

  // Target position goal for usv
  geometry_msgs::Point point;
  point.x = next_goal.x;
  point.y = next_goal.y;
  point.z = 0;

  // Publish next subgoal location while not achieved final goal location
  while (ros::ok()){
    
    // ROS_INFO("C_x - N_x: %d - %d --- C_y - N_y: %d - %d", usv_current_pos.x, next_goal.x, usv_current_pos.y, next_goal.y);
    // // Set new subgoal location after reach current subgoal
    // if((abs(usv_current_pos.x - next_goal.x) <= ACCEPTABLE_RADIUS) && (abs(usv_current_pos.y - next_goal.y) <= ACCEPTABLE_RADIUS)){ // Tests if achieved subgoal

    //   if(!path->empty()){

    //     next_goal = path->front();                                  // Gets next subgoal location
    //     path->pop();                                                // Removes from path data structure

    //     // Publish new subgoal
    //     point.x = next_goal.x;
    //     point.y = next_goal.y;
        // usv_guider_command_position_msg.pose.pose.position = point;
        // usv_guider_pub.publish(usv_guider_command_position_msg);

    //   }

    // }

    // Attending to subscription callbacks
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

std::queue<Pos>* A_Star(){

  std::queue<Pos> *path;

  path->push(Pos{200,60});
  path->push(Pos{20,20});

  return path;
}