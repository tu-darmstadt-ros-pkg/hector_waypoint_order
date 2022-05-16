#include "hector_waypoint_order/waypoint_order_computer_base.h"

#include <utility>

namespace hector_waypoint_order
{
void
WaypointOrderComputerBase::initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                      CostMap cost_map)
{
  nh_ = nh;
  waypoints_unordered_ = std::move(waypoints_unordered);

  if (waypoints_unordered_.empty())
  {
    throw std::runtime_error("WaypointOrderComputerBase: waypoints_unordered list was empty!");
  }

  cost_map_ = std::move(cost_map);

  path_costs_ = 0;
  path_.clear();
  num_steps_last_change_ = 0;

  publish_path_ = nh_.param("publish_path", false);

  if (publish_path_)
  {
    path_pub_ = nh_.advertise<nav_msgs::Path>("waypoint_order_computer_path", 1, true);
  }
  else
  {
    ROS_WARN_STREAM("No paths will be published by WaypointOrderComputer.");
  }
}

double WaypointOrderComputerBase::getPathCosts()
{
  // compute path costs if not set
  if (path_costs_ == 0 && path_.size() > 1)
  {
    path_costs_ = computePathCosts(path_);
  }

  return path_costs_;
}


int WaypointOrderComputerBase::getNumStepsOfLastChange()
{
  return num_steps_last_change_;
}


double WaypointOrderComputerBase::computePathCosts(std::vector<geometry_msgs::PoseStamped> path, double max_costs)
{
  double path_costs = 0;

  for (unsigned int i = 1; i < path.size(); ++i)
  {
    auto costs = cost_map_.at({path[i - 1], path[i]});
    if (costs == -1)
    {
      // TODO how to handle path_costs -1 in getPathCosts? Now set to DBL_MAX and break afterwards.
      path_costs = DBL_MAX;
      break;
    }
    path_costs += costs;

    // if max_costs are exceeded, break
    if (path_costs > max_costs)
    {
      path_costs = DBL_MAX;
      break;
    }
  }

  return path_costs;
}

void WaypointOrderComputerBase::publishPath()
{
  if (!publish_path_ || path_.empty())
  {
    return;
  }

  nav_msgs::Path path_msg;

  path_msg.header.frame_id = path_[0].header.frame_id;
  path_msg.header.stamp = ros::Time::now();

  path_msg.poses = path_;

  path_pub_.publish(path_msg);
}
} // end namespace hector_waypoint_order