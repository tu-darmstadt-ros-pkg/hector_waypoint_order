#include "hector_waypoint_order/tsp_solver/mst_tsp_solver.h"

#include <boost/graph/metric_tsp_approx.hpp>

#include <pluginlib/class_list_macros.h>

namespace hector_waypoint_order
{

void MstTspSolver::initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                              CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);

  graph_.clear();

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

      double costs;
      try
      {
        costs = cost_map_.at({waypoints_unordered_.at(i), waypoints_unordered_.at(j)});
      }
      catch (std::exception& e)
      {
        throw std::runtime_error("Error in MST TSP solver: cost map has no entry for {" +
                                 utils::to_string(waypoints_unordered_.at(i).pose.position) + ", " +
                                 utils::to_string(waypoints_unordered_.at(j).pose.position) + "}.");
      }


      if (costs == -1)
      {
        // Note: setting costs for non-computable paths to DBL_MAX results in discarding the edge in dijkstra_shortest_paths
        //  as here an maximum value for distances is set whose default is type::max.
        //  Another possibility would be to check (here or in cost computer) if this waypoint can be discarded completely due to unreachability.
        costs = DBL_MAX;
      }

      // add edge with costs from cost_map
      add_edge(vertex_i, vertex_j, costs, graph_);
    }
  }
}


std::vector<geometry_msgs::PoseStamped> MstTspSolver::computeWaypointOrder()
{

  // resize result vector
  std::vector<Graph::vertex_descriptor> tsp_path(num_vertices(graph_));

  // execute tsp solver
  // TODO metric_tsp_approx_tour can also get an element of the graph as start,
  //  in the current version the first element in the graph is used as start.
  //  Maybe add option to give pass first point as argument (but then for all order computer)
  //  (but with default which selects first or random as start).
  metric_tsp_approx_tour(graph_, back_inserter(tsp_path));


  // get vertex_index map for easier access
  auto vertex_index_map = get(boost::vertex_index, graph_);


  // convert tsp_path to vector of geometry_msgs::PoseStamped
  for (Graph::vertex_descriptor vertex_descr: tsp_path)
  {
    if (vertex_descr != Graph::null_vertex())
    {
      auto waypoint = waypoints_unordered_.at(vertex_index_map[vertex_descr]);

      path_.push_back(waypoint);
    }
  }

  publishPath();

  return path_;
}
} // end namespace hector_waypoint_order


PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::MstTspSolver, hector_waypoint_order::WaypointOrderComputerBase)