#include "ros/ros.h"                                  // ROS
#include "std_msgs/String.h"                          // ROS_INFO
#include <nav_msgs/Odometry.h>                        // usv_guider command msg

#include "../include/usv_guider/GridWithWeights.hpp"  // A*
#include "../include/usv_guider/PriorityQueue.hpp"    // A*
#include "../include/usv_guider/AStar.hpp"            // A*

#include <sstream>

void Test0_AStar(int, int, int, int);
void usv_state_callback(const nav_msgs::Odometry::ConstPtr&);

#define PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND 0.2
#define PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND  10
#define SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION       10

int main(int argc, char **argv)
{
  
  // Basic configuration to start node and configure a publisher
  ros::init(argc, argv, "usv_guider_node");                                                                                                         // Basic iniitalization for ROS node
  ros::NodeHandle n;                                                                                                                                // Handler to interact with ROS
  ros::Publisher usv_guider_pub = n.advertise<nav_msgs::Odometry>("/airboat/move_usv/goal", PUBLISH_QUEUE_AMOUNT___USV_GUIDER___WAYPOINT_COMMAND);  // Declaration of ROS topic and creation of a publishing handler for usv_guider waypoint command 
  ros::Rate loop_rate(PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND);                                                                       // Runs CA strategy each 1/PUBLISH_TIME_INTERVAL___USV_GUIDER___WAYPOINT_COMMAND secs.  

  ros::Subscriber usv_guider_sub = n.subscribe("/airboat/state", SUBSCRIBE_QUEUE_AMOUNT___USV___CURRENT_POSITION, usv_state_callback);

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

  // Pos next_goal = path->pop();

  while (ros::ok())
  {

    // while(1){ //while(usv.currentPos() != usv_guider.getFinalGoal())

    //   // if()

    // }

    
    ROS_INFO("Test >>>");
    usv_guider_pub.publish(usv_guider_command_position_msg);
    ros::spinOnce();
    loop_rate.sleep();

    Test0_AStar(-1, -1, -1, -1);

  }

  return 0;
}

// std::queue<Pos>* A_Star(){

//   std::queue<Pos> path;

//   path.push(Pos{0,0});

//   return &path;
// }

void Test0_AStar(int startX=-1, int startY=-1, int goalX=-1, int goalY=-1){

    GridWithWeights sg  = make_diagram4();

    Pos start {0, 0};
    Pos goal  {0, 0};

    if(startX > -1) start.x = startX;
    if(startY > -1) start.y = startY;
    if(goalX  > -1) goal.x  = goalX;
    if(goalY  > -1) goal.y  = goalY;

    std::unordered_map<Pos, Pos>    came_from;
    std::unordered_map<Pos, double> cost_so_far;

    AStar_Search(sg, start, goal, came_from, cost_so_far);

    draw_grid(sg, 2, nullptr, &came_from);
    std::cout << '\n';

    draw_grid(sg, 3, &cost_so_far, nullptr);
    std::cout << '\n';

    std::vector<Pos> path = reconstruct_path(start, goal, came_from);
    draw_grid(sg, 3, nullptr, nullptr, &path);

}

void usv_state_callback(const nav_msgs::Odometry::ConstPtr& usv_position_msg){

  ROS_INFO("USV current position: X: %f, Y: %f", usv_position_msg->pose.pose.position.x, usv_position_msg->pose.pose.position.y);

}