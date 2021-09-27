#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/GridMapMsgHelpers.hpp>

int width;
int height;
int value;
double resolution;
std::string frame_id;
std::string base_occupancy_map;
std::string base_grid_map;

ros::Publisher map_pub;

double position_x;
double position_y;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid new_map;

  new_map.header.frame_id = frame_id;
  new_map.info.width = width;
  new_map.info.height = height;
  new_map.info.resolution = resolution;
  new_map.info.origin.position.x = -((width+(width-msg->info.width*2))*resolution*0.25) + position_x;
  new_map.info.origin.position.y = -((height+(height-msg->info.height*2))*resolution*0.25) + position_y;
  new_map.info.origin.position.z = 0.0;
  new_map.info.origin.orientation.z = 0.0;
  new_map.info.origin.orientation.w = 1.0;
  new_map.data.assign(width * height, value);

  double h = height*resolution*0.5 - msg->info.height*resolution*0.5;

  int map_index = 0;
  int msg_index = 0;
  for (double i = 0; i < height; i++){
    for (double j = 0; j < width; j++){
      if(h<i*resolution && height*resolution-h>i*resolution){
        if(i*width+(width-msg->info.width)*0.5<=map_index && 
            (i+1)*width-(width-msg->info.width)*0.5>=map_index){
          new_map.data[map_index] = 
            msg->data[(j-(width-msg->info.width)*0.5) + (i-(height-msg->info.height)*0.5) * msg->info.width];
          msg_index++;
        }
      }
      map_index++;
    }
  }
  map_pub.publish(new_map);
}

void gridCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);
  
  map.getLength();
  grid_map::Position position = map.getPosition() - 0.5 * map.getLength().matrix();

  position_x = position.x();
  position_y = position.y();
}

void rosparam_set(ros::NodeHandle& pnh)
{
  pnh.param("map_width", width, 1500);
  pnh.param("map_height", height, 1000);
  pnh.param("map_value", value, -1);
  pnh.param("map_resolution", resolution, 0.1);
  pnh.param("map_frame_id", frame_id, static_cast<std::string>("odom"));
  pnh.param("base_occupancy_map_name", base_occupancy_map, static_cast<std::string>("/traversability_map_visualization/step_map"));
  pnh.param("base_grid_map_name", base_grid_map, static_cast<std::string>("/elevation_mapping/elevation_map"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancygrid_expander_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  rosparam_set(pnh);

  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancygrid_expander_map",10);
 
  ros::Subscriber map_sub = nh.subscribe(base_occupancy_map, 1 , mapCallback);
  ros::Subscriber grid_sub = nh.subscribe(base_grid_map, 1 , gridCallback);
  
  ros::spin();
  return 0;
}