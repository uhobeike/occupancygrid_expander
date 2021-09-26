#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>

std::vector<uint8_t> realMap;     //Real world map.

ros::Publisher map_pub;

int width =         1000;
int height =        1000;
float resolution =  0.1;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  ROS_INFO("Got map %d %d", info.width, info.height);

  nav_msgs::OccupancyGrid new_map;

  new_map.info.width = width;
  new_map.info.height = height;
  new_map.info.resolution = resolution;
  new_map.info.origin.position.x = -static_cast<double>(width) / 2 * resolution;
  new_map.info.origin.position.y = -static_cast<double>(height) / 2 * resolution;
  new_map.info.origin.orientation.w = 1.0;
  new_map.data.assign(width * height, -1);

  map_pub.publish(new_map);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "occupancygrid_expander_node");
  ros::NodeHandle n;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancygrid_expander_map",10);
  ros::Subscriber map_sub = n.subscribe("/traversability_map_visualization/step_map",1 , mapCallback);
  
  ros::spin();
  return 0;
}