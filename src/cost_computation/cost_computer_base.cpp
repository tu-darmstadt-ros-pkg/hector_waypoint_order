#include "hector_waypoint_order/cost_computation/cost_computer_base.h"

#include <utility>


namespace hector_waypoint_order
{

CostComputerBase::CostComputerBase(std::string plugin_name): plugin_name_(std::move(plugin_name))
{
}


void CostComputerBase::initialize(ros::NodeHandle& nh)
{
  pnh_ = nh;
}

const PathMap& CostComputerBase::getPaths()
{
  if(paths_.empty())
  {
    throw std::runtime_error("paths_ requested with getPaths() are either not computed yet or not available for CostComputer Plugin " + plugin_name_ + "!");
  }
  else
  {
    return paths_;
  }
}
} // end namespace hector_waypoint_order