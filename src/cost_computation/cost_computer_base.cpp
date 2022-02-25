#include "hector_waypoint_order/cost_computation/cost_computer_base.h"


namespace hector_waypoint_order
{


void CostComputerBase::initialize(ros::NodeHandle& nh)
{
  pnh_ = nh;
}


} // end namespace hector_waypoint_order