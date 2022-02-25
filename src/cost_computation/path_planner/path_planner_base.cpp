#include "hector_waypoint_order/cost_computation/path_planner/path_planner_base.h"


namespace hector_waypoint_order
{


void PathPlannerBase::initialize(ros::NodeHandle& nh)
{
  pnh_ = nh;
}

} // end namespace hector_waypoint_order