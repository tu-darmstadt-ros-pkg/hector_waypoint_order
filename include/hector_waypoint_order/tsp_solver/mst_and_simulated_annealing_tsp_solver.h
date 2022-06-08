#ifndef HECTOR_WAYPOINT_ORDER_MST_AND_SIMULATED_ANNEALING_TSP_SOLVER_H
#define HECTOR_WAYPOINT_ORDER_MST_AND_SIMULATED_ANNEALING_TSP_SOLVER_H


#include "hector_waypoint_order/waypoint_order_computer_base.h"

#include "hector_waypoint_order/tsp_solver/mst_tsp_solver.h"
#include "hector_waypoint_order/tsp_solver/simulated_annealing_tsp_solver.h"


namespace hector_waypoint_order
{

class MstAndSimulatedAnnealingTspSolver : public WaypointOrderComputerBase
{
public:

  void initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                  CostMap cost_map) override;

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() override;


private:

  MstTspSolver mst_tsp_solver_;
  SimulatedAnnealingTspSolver simulated_annealing_tsp_solver_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_MST_AND_SIMULATED_ANNEALING_TSP_SOLVER_H
