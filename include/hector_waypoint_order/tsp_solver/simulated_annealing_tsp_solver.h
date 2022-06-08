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
   * @return neighbor of current_path_
   */
  std::vector<geometry_msgs::PoseStamped> swap2Nodes();

  /**
   * Neighbor generation method (mutator): swap n edges
   * @param n number of edges to swap
   * @return neighbor of current_path_
   */
  std::vector<geometry_msgs::PoseStamped> swapNEdges(int n);

  /**
   * See swapNEdges.
   * @return neighbor of current_path_
   */
  std::vector<geometry_msgs::PoseStamped> swap2Edges();

  /**
   * See swapNEdges.
   * @return neighbor of current_path_
   */
  std::vector<geometry_msgs::PoseStamped> swap3Edges();


  /**
   * Neighbor generation method (mutator): move one node to another position (no swap!)
   * @return neighbor of current_path_
   */
  std::vector<geometry_msgs::PoseStamped> moveNode();


  /**
   * Computes the probability for accepting a worse neighbor based on the temperture_ and the costs of the current and the neighbor path.
   * If neighbor is better, this is always 1.
   * @param current_path_costs
   * @param neighbor_path_costs
   * @return
   */
  double computeProbabilityForAcceptingNeighbor(double current_path_costs, double neighbor_path_costs);

  /**
   * Compute next temperature based on current temperature and cooling schedule / cooling_rate_
   */
  void computeNextTemperature();


  void publishCurrentPath();

  /**
   * Only required for tests. Write statistics about the accepted path costs, all path costs and the temperatures to a file.
   */
  void writeStatisticsToFile();


  std::vector<geometry_msgs::PoseStamped> current_path_;

  std::vector<geometry_msgs::PoseStamped> neighbor_path_;

  double temperature_;
  double cooling_rate_;

  std::mt19937 probability_gen_;
  std::uniform_real_distribution<> probability_dis_;

  std::mt19937 neighbor_gen_;
  std::uniform_int_distribution<> neighbor_dis_;

  nav_msgs::Path current_path_msg_;
  ros::Publisher current_path_pub_;


  typedef std::function<std::vector<geometry_msgs::PoseStamped>(SimulatedAnnealingTspSolver&)> Mutator;
  std::map<int, Mutator> mutators_;

  bool use_best_mutator_;

  int max_tries_ = 1000;

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
