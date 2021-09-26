#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"

std::vector<uint8_t> realMap;     //Real world map.

ros::Publisher map_pub;

int width =         200;
int height =        100;
float resolution =  0.1;

double x;
double y;
std::string frame_id;

grid_map::Position    getPosition();

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  // ROS_INFO("Got map %d %d", info.width, info.height);

  nav_msgs::OccupancyGrid new_map;

  new_map.header.frame_id = frame_id;
  new_map.info.width = width;
  new_map.info.height = height;
  new_map.info.resolution = resolution;
  new_map.info.origin.position.x = x;
  new_map.info.origin.position.y = y;
  new_map.info.origin.position.z = 0.0;
  // new_map.info.origin.position.x = msg->info.origin.position.x;
  // new_map.info.origin.position.y = msg->info.origin.position.y;
  // new_map.info.origin.orientation.z = msg->info.origin.orientation.z;
  // new_map.info.origin.orientation.w = msg->info.origin.orientation.w;
  new_map.info.origin.orientation.z = 0.0;
  new_map.info.origin.orientation.w = 1.0;
  new_map.data.assign(width * height, -1);

  map_pub.publish(new_map);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // x = msg->pose.pose.orientation.z;
  // y = msg->pose.pose.orientation.w;
  // ROS_INFO("Got map %f %f", x, y);
}

void gridCallback(const grid_map_msgs::GridMap& msg){
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  
  map.getLength();
  grid_map::Position position = map.getPosition() - 0.5 * map.getLength().matrix();
  x = position.x();
  y = position.y();
  frame_id = map.getFrameId();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "occupancygrid_expander_node");
  ros::NodeHandle n;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancygrid_expander_map",10);
  ros::Subscriber map_sub = n.subscribe("/traversability_map_visualization/step_map",1 , mapCallback);
  ros::Subscriber odom_sub = n.subscribe("/t265/odom/sample",1 , odomCallback);
  ros::Subscriber grid_sub = n.subscribe("/elevation_mapping/elevation_map",1 , gridCallback);
  
  ros::spin();
  return 0;
}