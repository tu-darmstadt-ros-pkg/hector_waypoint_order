#ifndef HECTOR_WAYPOINT_ORDER_COST_COMPUTER_BASE_H
#define HECTOR_WAYPOINT_ORDER_COST_COMPUTER_BASE_H

#include "hector_waypoint_order/utils/utils.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


namespace hector_waypoint_order
{

class CostComputerBase
{
public:

  CostComputerBase() = default;

  explicit CostComputerBase(std::string plugin_name);

  virtual void initialize(ros::NodeHandle& nh);

  /**
   * Compute costs between each pair of waypoints from given waypoint list.
   * Resulting cost map contains for the waypoints A and B the pairs
   * (A,B), (B,A), (A,A), (B,B)
   * and the corresponding costs.
   * The costs might be -1, if they could not be computed (e.g. no path between them).
   * @param waypoints List of waypoints.
   * @return CostMap containing all pairs of waypoints with the costs between them.
   */
  virtual CostMap computeCosts(std::vector<geometry_msgs::PoseStamped> waypoints) = 0;

  /**
   * If during the cost computation paths have been computed and stored in paths_, they are returned here.
   * If paths_ is empty, a runtime error is thrown.
   * @return paths between waypoints
   */
  virtual const PathMap& getPaths();

protected:
  ros::NodeHandle pnh_;

  const std::string plugin_name_;

  PathMap paths_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_COST_COMPUTER_BASE_H
