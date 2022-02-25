#ifndef HECTOR_WAYPOINT_ORDER_PATH_PLANNER_BASE_H
#define HECTOR_WAYPOINT_ORDER_PATH_PLANNER_BASE_H

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace hector_waypoint_order
{

class PathPlannerBase
{
public:

  PathPlannerBase() = default;

  virtual void initialize(ros::NodeHandle& nh);

  /**
   * Plan a path from given start to target.
   * @param [in] start
   * @param [in] target
   * @param [out] path Computed path
   * @param [out] costs Costs (only if computed by path planner)
   * @return true if a path could be planned
   */
  virtual bool planPath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& target,
                        nav_msgs::Path& path, double& costs) = 0;

protected:

  ros::NodeHandle pnh_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_PATH_PLANNER_BASE_H
