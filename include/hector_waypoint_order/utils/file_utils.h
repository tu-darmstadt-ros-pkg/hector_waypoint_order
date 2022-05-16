#ifndef HECTOR_WAYPOINT_ORDER_FILE_UTILS_H
#define HECTOR_WAYPOINT_ORDER_FILE_UTILS_H

#include <hector_waypoint_order/Costmap.h>

#include <nav_msgs/Path.h>

#include <fstream>
#include <iterator>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ros/package.h>

namespace hector_waypoint_order
{

namespace file_utils
{

template<typename T>
static void writeVectorInFile(std::string file_name, std::vector<T> data)
{
  // if file name is not absolute, complete it
  file_name = (file_name[0] == '/') ? file_name : ros::package::getPath(ROS_PACKAGE_NAME) + "/data" + "/" + file_name;

  ROS_INFO_STREAM("Try to write statistics in file " << file_name);

  std::ofstream output_file(file_name);

  // write each entry in a new line
  std::string delimiter = "\n";

  std::ostream_iterator<T> output_iterator(output_file, delimiter.c_str());

  std::copy(data.begin(), data.end(), output_iterator);
}



template<typename T1, typename T2>
static void writeVectorInFile(std::string file_name, std::vector<std::pair<T1, T2>> data)
{
  // if file name is not absolute, complete it
  file_name = (file_name[0] == '/') ? file_name : ros::package::getPath(ROS_PACKAGE_NAME) + "/data" + "/" + file_name;

  ROS_INFO_STREAM("Try to write statistics in file " << file_name);

  std::ofstream output_file(file_name);

  for(auto& element: data)
  {
    output_file << element.first << " " << element.second << std::endl;
  }
}




static void
writeWaypointsAndCostMapToFile(const std::string& file_name, const std::vector<geometry_msgs::PoseStamped>& waypoints,
                               const CostMap& cost_map)
{
  rosbag::Bag bag;
  bag.open(file_name, rosbag::bagmode::Write);

  // convert cost_map to message
  Costmap cost_map_msg;
  cost_map_msg.cost_map.reserve(cost_map.size());
  for (auto& entry: cost_map)
  {
    // convert to entry
    CostMapEntry cost_map_entry;
    cost_map_entry.first_pose = entry.first.first;
    cost_map_entry.second_pose = entry.first.second;
    cost_map_entry.costs = entry.second;

    // add entry to message
    cost_map_msg.cost_map.push_back(cost_map_entry);
  }

  // convert waypoints to nav_msg/Path
  nav_msgs::Path waypoints_msg;
  waypoints_msg.poses = waypoints;

  // write messages to file
  bag.write("cost_map", ros::Time::now(), cost_map_msg);
  bag.write("waypoints", ros::Time::now(), waypoints_msg);

  bag.close();
}


static void
readWaypointsAndCostMapFromFile(const std::string& file_name, std::vector<geometry_msgs::PoseStamped>& waypoints,
                                CostMap& cost_map)
{

  rosbag::Bag bag;
  try
  {
    bag.open(file_name);
  }
  catch (rosbag::BagException& e)
  {
    ROS_ERROR_STREAM("Could not open bag file with name: " << file_name);
    throw e;
  }


  // define topics for view
  std::vector<std::string> topics = {"waypoints", "cost_map"};

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ROS_INFO_STREAM("Reading from bag file " << file_name << ".");

  for (rosbag::MessageInstance const msg: view)
  {
    ROS_INFO_STREAM("Read message on topic: " << msg.getTopic() << "with data type: " << msg.getDataType());

    // waypoints topic
    if (msg.getTopic() == topics[0])
    {
      nav_msgs::Path::ConstPtr waypoints_msg = msg.instantiate<nav_msgs::Path>();

      // check if it could be converted to requested msg type
      if (waypoints_msg != nullptr)
      {
        waypoints = waypoints_msg->poses;
      }
    }
      // costmap topic
    else if (msg.getTopic() == topics[1])
    {
      Costmap::ConstPtr cost_map_msg = msg.instantiate<Costmap>();

      // check if it could be converted to Costmap msg type
      if (cost_map_msg != nullptr)
      {
        // add all elements of CostMapEntry list to cost map
        for (auto& entry : cost_map_msg->cost_map)
        {
          auto result = cost_map.emplace(std::make_pair(entry.first_pose, entry.second_pose), entry.costs);
        }
      }
    }
  }

  bag.close();
}
} // end namespace file_utils

} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_FILE_UTILS_H
