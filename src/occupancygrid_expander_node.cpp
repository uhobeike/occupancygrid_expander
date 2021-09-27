#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"

std::vector<uint8_t> realMap;     //Real world map.

ros::Publisher map_pub;

int width =         400;
int height =        200;
float resolution =  0.1;

double x;
double y;
std::string frame_id;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  // ROS_INFO("Got map %d %d", info.width, info.height);

  nav_msgs::OccupancyGrid new_map;

  new_map.header.frame_id = "odom";
  new_map.info.width = width;
  new_map.info.height = height;
  new_map.info.resolution = resolution;
  new_map.info.origin.position.x = x;
  new_map.info.origin.position.y = y;
  new_map.info.origin.position.z = 0.0;
  new_map.info.origin.orientation.z = 0.0;
  new_map.info.origin.orientation.w = 1.0;
  new_map.data.assign(width * height, -1);

  double h = height*resolution*0.5 - msg->info.height*resolution*0.5;
  // std::cout << height*resolution*0.5 << ", " << msg->info.height*resolution*0.5 << ", " << h << "\n";

  int cnt = 0;
  // std::vector<int8_t> msg_data;
  // msg_data.resize(msg->info.width*msg->info.height);
  // for (int i = 0; i < msg->info.width; i++){
  //   for (int j = 0; j < msg->info.height; j++){
  //     std::cout << msg->data[i + msg->info.width *j];
  //     msg_data.push_back(msg->data[cnt++]);
  //   }
  // }
  int map_index = 0;
  int msg_index = 0;
  for (double i = 0; i < height; i++){
    for (double j = 0; j < width; j++){
      if(h<i*resolution && height*resolution-h>i*resolution){
        if(i*width+(width-msg->info.width)*0.5<=map_index && 
            (i+1)*width-(width-msg->info.width)*0.5>=map_index){
          new_map.data[map_index] = msg->data[(j-100) + ((i-50)) * msg->info.width];
          // std::cout << (j-100) + (msg->info.height - (i-50) - 1) * msg->info.width << "\n";
          msg_index++;
        }
      }
      map_index++;
      // std::cout << j*resolution << "\n";
    }
  }
  map_pub.publish(new_map);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // x = msg->pose.pose.orientation.z;
  // y = msg->pose.pose.orientation.w;
  // ROS_INFO("Got map %f %f", x, y);
}

void gridCallback(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map);
  
  map.getLength();
  grid_map::Position position = map.getPosition() - 0.5 * map.getLength().matrix();
  x = -(width*resolution*0.25) + position.x();
  y = -(height*resolution*0.25) + position.y();
  // frame_id = map.getFrameId();
  // std::cout << x << y << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancygrid_expander_node");
  ros::NodeHandle n;

  map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancygrid_expander_map",10);
  ros::Subscriber map_sub = n.subscribe("/traversability_map_visualization/step_map",1 , mapCallback);
  ros::Subscriber odom_sub = n.subscribe("/t265/odom/sample",1 , odomCallback);
  ros::Subscriber grid_sub = n.subscribe("/elevation_mapping/elevation_map",1 , gridCallback);
  
  ros::spin();
  return 0;
}