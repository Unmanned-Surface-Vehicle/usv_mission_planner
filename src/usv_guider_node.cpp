#include "ros/ros.h"
#include "std_msgs/String.h"

#include "../include/usv_guider/GridWithWeights.hpp"
#include "../include/usv_guider/PriorityQueue.hpp"
#include "../include/usv_guider/AStar.hpp"

#include <sstream>

void Test0_AStar(int, int, int, int);

int main(int argc, char **argv)
{

  /*
  * Basic configuration to start node and configure a publisher
  */
  ros::init(argc, argv, "usv_guider_node");                                                   // Basic iniitalization for ROS node
  ros::NodeHandle n;                                                                          // Handler to interact with ROS
  ros::Publisher usv_guider_pub = n.advertise<std_msgs::String>("usv_guider_command", 1000);  // Declaration of ROS topic and creation of a publishing handler
  ros::Rate loop_rate(0.2);                                                                    // Runs CA strategy each 5 secs.

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    usv_guider_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

    Test0_AStar(-1, -1, -1, -1);

  }


  return 0;
}

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