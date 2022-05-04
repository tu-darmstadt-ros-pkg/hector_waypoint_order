#ifndef HECTOR_WAYPOINT_ORDER_SIMULATED_ANNEALING_TSP_SOLVER_H
#define HECTOR_WAYPOINT_ORDER_SIMULATED_ANNEALING_TSP_SOLVER_H

#include "hector_waypoint_order/waypoint_order_computer_base.h"

#include <random>


namespace hector_waypoint_order
{

class SimulatedAnnealingTspSolver : public WaypointOrderComputerBase
{
public:

  void initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                  CostMap cost_map) override;

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() override;

  /**
   * Set initial state for simulated annealing. If this method is not called, the waypoints_unordered_ list is used as initial state.
   * @param initial_path
   */
  void setInitialPath(std::vector<geometry_msgs::PoseStamped> initial_path);

private:

  /**
   * picks a random neighbor
   * @return neighbor costs
   */
  double pickRandomNeighbor();


  /**
   * Neighbor generation method (mutator): swap two nodes
   */
  std::vector<geometry_msgs::PoseStamped> swap2Nodes();

  /**
   * Neighbor generation method (mutator): swap n edges
   * @param n number of edges to swap
   */
  std::vector<geometry_msgs::PoseStamped> swapNEdges(int n);

  std::vector<geometry_msgs::PoseStamped> swap2Edges();
  std::vector<geometry_msgs::PoseStamped> swap3Edges();



  /**
   * Neighbor generation method (mutator): move one node to another position (no swap!)
   */
  std::vector<geometry_msgs::PoseStamped> moveNode();


  double computeProbabilityForAcceptingNeighbor(double current_path_costs, double neighbor_path_costs);

  void computeNextTemperature();


  void publishCurrentPath();

  void writeStatisticsToFile();


  std::vector<geometry_msgs::PoseStamped> current_path_;

  std::vector<geometry_msgs::PoseStamped> neighbor_path_;

  double temperature_;

  std::mt19937 probability_gen_;
  std::uniform_real_distribution<> probability_dis_;

  std::mt19937 neighbor_gen_;
  std::uniform_int_distribution<> neighbor_dis_;

  nav_msgs::Path current_path_msg_;
  ros::Publisher current_path_pub_;


  typedef std::function<std::vector<geometry_msgs::PoseStamped>(SimulatedAnnealingTspSolver&)> Mutator;
  std::map<int, Mutator> mutators_;

  bool use_best_mutator_;

  const int MOVE_NODE = 0;
  const int SWAP_2_NODES = 1;
  const int SWAP_2_EDGES = 2;
  const int SWAP_3_EDGES = 3;



  // statistics
  std::vector<double> accepted_path_costs_statistics_;
  std::vector<double> all_path_costs_statistics_;
  std::vector<double> temperature_statistics_;

  int mutator_statistics_[4] = {0, 0, 0, 0};
  int mutator_accepted_statistics_[4] = {0, 0, 0, 0};
  int used_mutator_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_SIMULATED_ANNEALING_TSP_SOLVER_H
