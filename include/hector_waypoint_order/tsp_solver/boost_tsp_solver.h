#ifndef HECTOR_WAYPOINT_ORDER_BOOST_TSP_SOLVER_H
#define HECTOR_WAYPOINT_ORDER_BOOST_TSP_SOLVER_H

#include "hector_waypoint_order/waypoint_order_computer_base.h"

#include <boost/graph/adjacency_list.hpp>

namespace hector_waypoint_order
{

class BoostTspSolver : public WaypointOrderComputerBase
{
public:

  void initialize(std::vector<geometry_msgs::PoseStamped> waypoints_unordered, CostMap cost_map) override;

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() override;

private:

  // Template parameters see https://www.boost.org/doc/libs/1_78_0/libs/graph/doc/adjacency_list.html
  using Graph = boost::adjacency_list<
    boost::setS, // edge container for outgoing edges on each vertex (use set to disallow parallel edges)
    boost::listS, // vertex container in graph
    boost::undirectedS, // directedS / undirectedS / bidirectionalS
    boost::property<boost::vertex_index_t, int>, // vertex property (here an index is stored instead of e.g. the waypoint position)
    boost::property<boost::edge_weight_t, double>, // edge property (weight = costs)
    boost::no_property, // graph property
    boost::listS // edge container in graph
  >;


  Graph graph_;


};
} // end namespace hector_waypoint_order


#endif //HECTOR_WAYPOINT_ORDER_BOOST_TSP_SOLVER_H
