
#include <ros/ros.h>

#include <hector_waypoint_order/waypoint_order_computer_base.h>

#include <hector_waypoint_order/tsp_solver/boost_tsp_solver.h>
#include <hector_waypoint_order/tsp_solver/simulated_annealing_tsp_solver.h>
#include <hector_waypoint_order/tsp_solver/boost_and_simulated_annealing_tsp_solver.h>
#include <hector_waypoint_order/tsp_solver/greedy_tsp_solver.h>
#include <hector_waypoint_order/tsp_solver/greedy_and_simulated_annealing_tsp_solver.h>

#include <hector_waypoint_order/utils/file_utils.h>

#include <visualization_msgs/Marker.h>

#include <pluginlib/class_loader.h>


using namespace hector_waypoint_order;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_tsp_solver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // load waypoints and cost map from file
  std::string file_name = pnh.param<std::string>("tsp_file_name", "tsp_waypoints_costmap.bag");
  file_name = (file_name[0] == '/') ? file_name :
              ros::package::getPath(ROS_PACKAGE_NAME) + "/test/test_node/data" + "/" + file_name;

  std::vector<geometry_msgs::PoseStamped> waypoints_unordered;
  CostMap cost_map;
  file_utils::readWaypointsAndCostMapFromFile(file_name, waypoints_unordered, cost_map);

  ROS_INFO_STREAM(
    "Bag file successfully read. Waypoints_unordered size: " << waypoints_unordered.size() << ", cost_map size: "
                                                             << cost_map.size() << "\n");

  // Init solver
  pluginlib::ClassLoader<WaypointOrderComputerBase> order_computer_loader_(
    "hector_waypoint_order",
    "hector_waypoint_order::WaypointOrderComputerBase");

  std::string order_computer_name = pnh.param<std::string>("waypoint_order_computer_plugin",
                                                           "hector_waypoint_order::BoostTspSolver");
  std::unique_ptr<WaypointOrderComputerBase> order_computer_;
  order_computer_.reset(order_computer_loader_.createUnmanagedInstance(order_computer_name));

  order_computer_->initialize(pnh, waypoints_unordered, cost_map);

  // publish waypoints
  ros::Publisher waypoints_pub = pnh.advertise<visualization_msgs::Marker>("waypoints", 1, true);

  visualization_msgs::Marker waypoint_marker_msg;
  waypoint_marker_msg.header.frame_id = "building";
  waypoint_marker_msg.type = visualization_msgs::Marker::SPHERE_LIST;
  waypoint_marker_msg.action = visualization_msgs::Marker::ADD;
  waypoint_marker_msg.scale.x = 0.25;
  waypoint_marker_msg.scale.y = 0.25;

  waypoint_marker_msg.points.reserve(waypoints_unordered.size());
  waypoint_marker_msg.colors.reserve(waypoints_unordered.size());
  std_msgs::ColorRGBA color;
  color.a = 1;
  color.r = 1.0;
  for (auto& wp: waypoints_unordered)
  {
    waypoint_marker_msg.points.push_back(wp.pose.position);
    waypoint_marker_msg.colors.push_back(color);
  }

  waypoints_pub.publish(waypoint_marker_msg);


  // start async spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // wait a few seconds so that model etc. is loaded in visualization
  sleep(3);

  // start solver
  order_computer_->computeWaypointOrder();

  ROS_INFO_STREAM("Final path costs: " << order_computer_->getPathCosts());


  ros::waitForShutdown();

  return 0;
}