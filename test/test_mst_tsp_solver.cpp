
#include "hector_waypoint_order/tsp_solver/mst_tsp_solver.h"

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <event2/event.h>


using namespace hector_waypoint_order;

class MstTspSolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    waypoints_unordered_.push_back(setPose(0, 0, 0));
    waypoints_unordered_.push_back(setPose(0, 10, 0));
    waypoints_unordered_.push_back(setPose(7, 9, 0));
    waypoints_unordered_.push_back(setPose(8, 0, 0));
    waypoints_unordered_.push_back(setPose(4, 5, 0));

    addCosts(0, 1, 10);
    addCosts(0, 2, -1);
    addCosts(0, 3, 8);
    addCosts(0, 4, 10);

    addCosts(1, 2, 7);
    addCosts(1, 3, -1);
    addCosts(1, 4, 11);

    addCosts(2, 3, 9);
    addCosts(2, 4, 8);

    addCosts(3, 4, 5);

    optimal_path_.push_back(waypoints_unordered_[0]);
    optimal_path_.push_back(waypoints_unordered_[1]);
    optimal_path_.push_back(waypoints_unordered_[2]);
    optimal_path_.push_back(waypoints_unordered_[4]);
    optimal_path_.push_back(waypoints_unordered_[3]);
    optimal_path_.push_back(waypoints_unordered_[0]);

    std::reverse(optimal_path_.begin(), optimal_path_.end());

    optimal_path_costs_ =
      cost_map_[{waypoints_unordered_[0], waypoints_unordered_[1]}] +
      cost_map_[{waypoints_unordered_[1], waypoints_unordered_[2]}] +
      cost_map_[{waypoints_unordered_[2], waypoints_unordered_[4]}] +
      cost_map_[{waypoints_unordered_[4], waypoints_unordered_[3]}] +
      cost_map_[{waypoints_unordered_[3], waypoints_unordered_[0]}];
  }

  static geometry_msgs::PoseStamped setPose(double x, double y, double z)
  {
    geometry_msgs::PoseStamped output;

    // set orientation to identity
    output.pose.orientation.w = 1;

    output.pose.position.x = x;
    output.pose.position.y = y;
    output.pose.position.z = z;

    return output;
  }

  void addCosts(int idx_a, int idx_b, double costs)
  {
    cost_map_[{waypoints_unordered_[idx_a], waypoints_unordered_[idx_b]}] = costs;
    cost_map_[{waypoints_unordered_[idx_b], waypoints_unordered_[idx_a]}] = costs;
  }

  std::vector<geometry_msgs::PoseStamped> waypoints_unordered_;

  std::vector<geometry_msgs::PoseStamped> optimal_path_;
  double optimal_path_costs_;

  CostMap cost_map_;
};


TEST_F(MstTspSolverTest, SmallTest)
{
  MstTspSolver solver;

  ros::NodeHandle nh;
  solver.initialize(nh, waypoints_unordered_, cost_map_);

  auto result = solver.computeWaypointOrder();

  for (const auto& waypoint: result)
  {
    ROS_WARN_STREAM("Path position: \n" << waypoint.pose.position);
  }

  EXPECT_EQ(result.size(), optimal_path_.size());

  EXPECT_EQ(solver.getPathCosts(), optimal_path_costs_);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_mst_tsp_solver");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}