/*To do
・Fixed a problem that values ​​cannot be
referenced properly when the map size is odd.
・Allows you to create an expanded map by synthesizing
multiple maps instead of just one map.
・Get the map location information from another
(tf or something?) Instead of getting it from the
elevation grid map.
・Since it is published at 1Hz, it will be possible to
pub an extended map at about 5Hz, which is faster.
・There may be some bugs.*/

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

int g_width;
int g_height;
int g_value;
double g_resolution;
std::string g_frame_id;
std::string g_base_occupancy_map;
std::string g_base_grid_map;

ros::Publisher g_map_pub;

double g_position_x;
double g_position_y;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  nav_msgs::OccupancyGrid new_map;

  new_map.header.frame_id = g_frame_id;
  new_map.info.width = g_width;
  new_map.info.height = g_height;
  new_map.info.resolution = g_resolution;
  new_map.info.origin.position.x =
      -((g_width + (g_width - msg->info.width * 2)) * g_resolution / 4) + g_position_x;
  new_map.info.origin.position.y =
      -((g_height + (g_height - msg->info.height * 2)) * g_resolution / 4) + g_position_y;
  new_map.info.origin.position.z = 0.0;
  new_map.info.origin.orientation.z = 0.0;
  new_map.info.origin.orientation.w = 1.0;
  new_map.data.assign(g_width * g_height, g_value);

  const double h = g_height * g_resolution / 2 - msg->info.height * g_resolution / 2;

  int map_index = 0;
  for (int i = 0; i < g_height; ++i)
  {
    for (int j = 0; j < g_width; ++j)
    {
      if (h < i * g_resolution && (g_height * g_resolution) - h > i * g_resolution)
      {
        if (i * g_width + (g_width - msg->info.width) / 2 <= map_index &&
            (i + 1) * g_width - (g_width - msg->info.width) / 2 >= map_index)
        {
          new_map.data[map_index] =
              msg->data[(j - (g_width - msg->info.width) / 2) +
                        (i - (g_height - msg->info.height) / 2) * msg->info.width];
        }
      }
      ++map_index;
    }
  }
  g_map_pub.publish(new_map);
}

void gridCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);
  map.getLength();
  grid_map::Position position = map.getPosition() - 0.5 * map.getLength().matrix();

  g_position_x = position.x();
  g_position_y = position.y();
}

void setRosparam(ros::NodeHandle& pnh)
{
  pnh.param("map_width", g_width, 1500);
  pnh.param("map_height", g_height, 1000);
  pnh.param("map_value", g_value, -1);
  pnh.param("map_resolution", g_resolution, 0.1);
  pnh.param("map_frame_id", g_frame_id, std::string("odom"));
  pnh.param("base_occupancy_map_name", g_base_occupancy_map,
            std::string("/traversability_map_visualization/step_map"));
  pnh.param("base_grid_map_name", g_base_grid_map, std::string("/elevation_mapping/elevation_map"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancygrid_expander_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  setRosparam(pnh);

  g_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("occupancygrid_expander_map", 1);

  ros::Subscriber map_sub = nh.subscribe(g_base_occupancy_map, 1, mapCallback);
  ros::Subscriber grid_sub = nh.subscribe(g_base_grid_map, 1, gridCallback);

  ros::spin();
  return 0;
}