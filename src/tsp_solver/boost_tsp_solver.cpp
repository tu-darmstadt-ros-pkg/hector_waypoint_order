#include "hector_waypoint_order/tsp_solver/boost_tsp_solver.h"

#include <boost/graph/metric_tsp_approx.hpp>

#include <pluginlib/class_list_macros.h>

namespace hector_waypoint_order
{

void BoostTspSolver::initialize(std::vector<geometry_msgs::PoseStamped> waypoints_unordered, CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(waypoints_unordered, cost_map);

  // add all vertices
  for (int i = 0; i < waypoints_unordered_.size(); ++i)
  {
    add_vertex(i, graph_);
  }

  // add edges, as it is an undirected graph, only the higher vertices are required for connections
  for (int i = 0; i < waypoints_unordered_.size(); ++i)
  {
    // get vertex i
    Graph::vertex_descriptor vertex_i = vertex(i, graph_);

    for (int j = i + 1; j < waypoints_unordered_.size(); ++j)
    {
      // get vertex j
      Graph::vertex_descriptor vertex_j = vertex(j, graph_);

      auto costs = cost_map_.at({waypoints_unordered_.at(i), waypoints_unordered_.at(j)});

      if (costs == -1)
      {
        // TODO non computable costs = DBL_MAX ok?
        costs = DBL_MAX;
      }

      // add edge with costs from cost_map
      add_edge(vertex_i, vertex_j, costs, graph_);
    }
  }
}


std::vector<geometry_msgs::PoseStamped> BoostTspSolver::computeWaypointOrder()
{

  // resize result vector
  std::vector<Graph::vertex_descriptor> tsp_path(num_vertices(graph_));

  // execute tsp solver
  metric_tsp_approx_tour(graph_, back_inserter(tsp_path));


  // get vertex_index map for easier access
  auto vertex_index_map = get(boost::vertex_index, graph_);


  // convert tsp_path to vector of geometry_msgs::PoseStamped
  for (Graph::vertex_descriptor vertex_descr: tsp_path)
  {
    // TODO: is this check required?
    if (vertex_descr != Graph::null_vertex())
    {
      auto waypoint = waypoints_unordered_.at(vertex_index_map[vertex_descr]);

      path_.push_back(waypoint);
    }
  }

  return path_;
}
} // end namespace hector_waypoint_order


PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::BoostTspSolver, hector_waypoint_order::WaypointOrderComputerBase)