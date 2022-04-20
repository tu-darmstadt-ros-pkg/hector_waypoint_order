#include "hector_waypoint_order/cost_computation/path_cost_computer.h"

#include <nav_msgs/Path.h>

#include <pluginlib/class_list_macros.h>


namespace hector_waypoint_order
{

PathCostComputer::PathCostComputer() : CostComputerBase("PathCostComputer")
{
}

void PathCostComputer::initialize(ros::NodeHandle& nh)
{
  CostComputerBase::initialize(nh);

  // init path planner (loader and plugin)
  std::string path_planner_name = pnh_.param<std::string>("path_planner_plugin", "");
  std::string path_planner_package_name = path_planner_name.substr(0, path_planner_name.find(':'));

  path_planner_loader_ptr_ = std::make_unique<pluginlib::ClassLoader<hector_waypoint_order::PathPlannerBase>>(
    path_planner_package_name, "hector_waypoint_order::PathPlannerBase");

  path_planner_.reset(path_planner_loader_ptr_->createUnmanagedInstance(path_planner_name));
  path_planner_->initialize(pnh_);
}


CostMap PathCostComputer::computeCosts(std::vector<geometry_msgs::PoseStamped> waypoints)
{

  CostMap cost_map;

  for (unsigned int i = 0; i < waypoints.size(); i++)
  {
    if(!ros::ok())
    {
      break;
    }

    auto start_pose = waypoints[i];

    // add entry of start_pose to itself with 0
    cost_map.emplace(std::make_pair(start_pose, start_pose), 0);

    for (unsigned int j = i + 1; j < waypoints.size(); j++)
    {
      if(!ros::ok())
      {
        break;
      }

      auto target_pose = waypoints[j];

      // TODO maybe do not compute costs between all pairs of waypoints
      //  e.g. if euclidean distance is over threshold, set distance to DBL_MAX?


      double costs;
      nav_msgs::Path path;

      if (!path_planner_->planPath(start_pose, target_pose, path, costs))
      {
        // if planner could not plan a path between start and target pose, set costs to -1
        // (not to inf/DBL_MAX to distinct between long but (probably) existing paths and non-existing path)
        costs = -1;

        path.poses.clear();
        path.poses.push_back(start_pose);
        path.poses.push_back(target_pose);
      }

      // if costs were not computed by path planner, compute them as path length
      if (costs == 0)
      {
        costs = computePathLength(path.poses);
      }

      // add paths to map
      paths_.emplace(std::make_pair(start_pose, target_pose), path);

      // reverse path for target -> start
      std::reverse(path.poses.begin(), path.poses.end());
      paths_.emplace(std::make_pair(target_pose, start_pose), path);

      // add costs for both directions to cost map
      cost_map.emplace(std::make_pair(start_pose, target_pose), costs);
      cost_map.emplace(std::make_pair(target_pose, start_pose), costs);

      // print progress
      int progress = static_cast<int>(std::round(cost_map.size() * 100.0 / (waypoints.size() * waypoints.size())));
      std::cout << "Compute path costs: " << progress << "%.\r";
      std::cout.flush();
    }
  }

  return cost_map;
}


double PathCostComputer::computePathLength(const std::vector<geometry_msgs::PoseStamped>& path)
{
  if (path.empty())
  {
    return 0;
  }

  double path_length = 0;
  geometry_msgs::PoseStamped previous, current;

  previous = path[0];

  // sum up euclidean distances between waypoints waypoints
  for (unsigned int i = 1; i < path.size(); i++)
  {
    current = path[i];

    if (previous.header.frame_id != current.header.frame_id)
    {
      throw std::logic_error(
        "Tried to compute path_length between two poses in different frames! previous: " +
        previous.header.frame_id + ", current: " + current.header.frame_id + ".");
    }

    path_length += std::sqrt(
      std::pow(previous.pose.position.x - current.pose.position.x, 2) +
      std::pow(previous.pose.position.y - current.pose.position.y, 2) +
      std::pow(previous.pose.position.z - current.pose.position.z, 2)
    );

    previous = current;
  }

  return path_length;
}
} // end namespace hector_waypoint_order


PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::PathCostComputer, hector_waypoint_order::CostComputerBase)