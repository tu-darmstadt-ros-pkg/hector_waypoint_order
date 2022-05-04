#include "hector_waypoint_order/tsp_solver/simulated_annealing_tsp_solver.h"

#include "hector_waypoint_order/utils/file_utils.h"

#include <utility>

#include <pluginlib/class_list_macros.h>


namespace hector_waypoint_order
{


void SimulatedAnnealingTspSolver::initialize(ros::NodeHandle& nh,
                                             std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                             CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);

  // set initial state (this can be overwritten by setInitialPath)
  current_path_ = waypoints_unordered_;
  current_path_.push_back(current_path_[0]);

  // TODO set appropriate initial temperature (evaluate to find best and maybe get from parameter server?)
  temperature_ = 5;


  // initialize random generators
  std::random_device rd1;
  probability_gen_ = std::mt19937(rd1());
  probability_dis_ = std::uniform_real_distribution<>(0.0, 1.0);


  std::random_device rd2;
  neighbor_gen_ = std::mt19937(rd2());
  // use only range [1,size-2] for neighbor index generator in order to ensure that it remains a circular path with path[0] == path[size-1]
  // (= never swap first or last element = start/end remains same for all neighbors)
  neighbor_dis_ = std::uniform_int_distribution<>(1, static_cast<int>(current_path_.size()) - 2);

  if (publish_path_)
  {
    current_path_pub_ = nh_.advertise<nav_msgs::Path>("simulated_annealing_current_path", 1, true);
    current_path_msg_.header.frame_id = current_path_[0].header.frame_id;
  }

  // add mutators to vector
  mutators_[MOVE_NODE] = &SimulatedAnnealingTspSolver::moveNode;
  mutators_[SWAP_2_NODES] = &SimulatedAnnealingTspSolver::swap2Nodes;
  mutators_[SWAP_2_EDGES] = &SimulatedAnnealingTspSolver::swap2Edges;
  mutators_[SWAP_3_EDGES] = &SimulatedAnnealingTspSolver::swap3Edges;

  use_best_mutator_ = nh_.param("use_best_mutator", true);
}


std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::computeWaypointOrder()
{
  // initialize for current path
  double current_path_costs = computePathCosts(current_path_);
  double neighbor_path_costs;

  accepted_path_costs_statistics_.push_back(current_path_costs);
  publishCurrentPath();

  // TEMPORARY sleep after visualization of initial solution
  sleep(1);

  // set current best solution
  path_ = current_path_;
  path_costs_ = current_path_costs;
  publishPath();


  // TODO set appropriate termination condition (maybe get from parameter server?)
//  double t_end = 0.005;
//  while (temperature_ >= t_end)

  int num_steps = std::pow(10, 5);
  int step = 0;
  while (step < num_steps)
  {
    // select neighbor and compute path costs of it
    neighbor_path_costs = pickRandomNeighbor();

    // compute acceptance probability
    double probability_accept_neighbor = computeProbabilityForAcceptingNeighbor(current_path_costs,
                                                                                neighbor_path_costs);

    // accept neighbor if the path costs are lower than the current ones or accept with the computed probability
    if (neighbor_path_costs < current_path_costs || probability_dis_(probability_gen_) < probability_accept_neighbor)
    {
      // TEMPORARY print
      ROS_INFO_STREAM(
        "Accepted neighbor path costs: " << neighbor_path_costs << ", current path costs: " << current_path_costs
                                         << ", probability: " << probability_accept_neighbor << ", used mutator: " << used_mutator_);

      current_path_ = neighbor_path_;
      current_path_costs = neighbor_path_costs;

      accepted_path_costs_statistics_.push_back(current_path_costs);
      ++mutator_accepted_statistics_[used_mutator_];
      publishCurrentPath();



      // update current best solution
      // (in order to avoid that a "worse" solution is used as the final solution if the termination condition breaks
      //  while a worse solution was previously accepted)
      if (current_path_costs < path_costs_)
      {
        path_ = current_path_;
        path_costs_ = current_path_costs;
        ROS_WARN_STREAM("Current path is shortest. Costs: " << path_costs_);
        publishPath();
      }

      // TEMPORARY sleep
      ros::Duration d(0.25);
      d.sleep();
    }

    // update temperature (in all steps not only when accepted)
    computeNextTemperature();

    step++;
  }


  // TEMPORARY write statistics to file
  writeStatisticsToFile();

  double sum_generated = std::accumulate(std::begin(mutator_statistics_), std::end(mutator_statistics_), 0);

  // TEMPORARY statistics print
  ROS_INFO_STREAM("Generated mutators:\n"
                    << "move node:    " << mutator_statistics_[MOVE_NODE] / sum_generated << "%\n"
                    << "swap 2 nodes: " << mutator_statistics_[SWAP_2_NODES] / sum_generated << "%\n"
                    << "swap 2 edges: " << mutator_statistics_[SWAP_2_EDGES] / sum_generated << "%\n"
                    << "swap 3 edges: " << mutator_statistics_[SWAP_3_EDGES] / sum_generated << "%\n");

  double sum_accepted = std::accumulate(std::begin(mutator_accepted_statistics_),
                                        std::end(mutator_accepted_statistics_), 0);

  // TEMPORARY statistics print
  ROS_INFO_STREAM("Accepted neighbors with mutator:\n"
                    << "move node:    " << mutator_accepted_statistics_[MOVE_NODE] / sum_accepted << "%\n"
                    << "swap 2 nodes: " << mutator_accepted_statistics_[SWAP_2_NODES] / sum_accepted << "%\n"
                    << "swap 2 edges: " << mutator_accepted_statistics_[SWAP_2_EDGES] / sum_accepted << "%\n"
                    << "swap 3 edges: " << mutator_accepted_statistics_[SWAP_3_EDGES] / sum_accepted << "%\n");


  // path was set in loop as best ever seen solution
  return path_;
}

void SimulatedAnnealingTspSolver::setInitialPath(std::vector<geometry_msgs::PoseStamped> initial_path)
{
  current_path_ = std::move(initial_path);

  // ensure that it is a circular path
  if (current_path_[0] != current_path_.back())
  {
    current_path_.push_back(current_path_[0]);
  }
}


double SimulatedAnnealingTspSolver::pickRandomNeighbor()
{
  std::vector<geometry_msgs::PoseStamped> neighbor;
  double neighbor_costs;

  if (use_best_mutator_)
  {
    neighbor_costs = DBL_MAX;

    std::vector<geometry_msgs::PoseStamped> candidate;
    double candidate_costs;

    for (auto& mutator: mutators_)
    {
      candidate = mutator.second(*this);
      candidate_costs = computePathCosts(candidate);

      if (candidate_costs < neighbor_costs)
      {
        used_mutator_ = mutator.first;
        neighbor = candidate;
        neighbor_costs = candidate_costs;
      }
    }
  }
  else
  {
    // mutate neighbor_path using different strategies

    // TODO evaluate to find appropriate probabilities for mutator strategies (parameter server?)
    double prob_move_node = 0.25;
    double prob_swap_nodes = 0.25;
    double prob_swap_2_edges = 0.25;
    double prob_swap_3_edges = 0.25;

    std::map<int, std::pair<double, double>> probabilities;
    probabilities[MOVE_NODE] = std::make_pair(0, prob_move_node);
    probabilities[SWAP_2_NODES] = std::make_pair(probabilities[MOVE_NODE].second,
                                                 probabilities[MOVE_NODE].second + prob_swap_nodes);
    probabilities[SWAP_2_EDGES] = std::make_pair(probabilities[SWAP_2_NODES].second,
                                                 probabilities[SWAP_2_NODES].second + prob_swap_2_edges);
    probabilities[SWAP_3_EDGES] = std::make_pair(probabilities[SWAP_2_EDGES].second,
                                                 probabilities[SWAP_2_EDGES].second + prob_swap_3_edges);

    if(probabilities[SWAP_3_EDGES].second != 1)
    {
      throw std::invalid_argument("TSP Solver Simulated Annealing: Mutator probabilities do not sum up to 1!");
    }

    auto random = probability_dis_(probability_gen_);

    for (auto& prob: probabilities)
    {
      if (prob.second.first <= random && random <= prob.second.second)
      {
        used_mutator_ = prob.first;
        neighbor = mutators_[prob.first](*this);
        neighbor_costs = computePathCosts(neighbor);
        break;
      }
    }
  }

  ++mutator_statistics_[used_mutator_];

  neighbor_path_ = neighbor;
  return neighbor_costs;
}


std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::swap2Nodes()
{
  std::vector<geometry_msgs::PoseStamped> neighbor = current_path_;

  bool indices_valid;
  int idx1, idx2;
  do
  {
    // randomly select two points in path to swap
    idx1 = neighbor_dis_(neighbor_gen_);
    idx2 = neighbor_dis_(neighbor_gen_);

    if (idx1 == idx2)
    {
      indices_valid = false;
      continue;
    }

    // get new edges after swap
    utils::PosePair new_edges[4] = {{neighbor[idx2 - 1], neighbor[idx1]},
                                    {neighbor[idx1],     neighbor[idx2 + 1]},
                                    {neighbor[idx1 - 1], neighbor[idx2]},
                                    {neighbor[idx2],     neighbor[idx1 + 1]}};

    indices_valid = true;
    for (auto& new_edge: new_edges)
    {
      // check for each new edge, if it is valid (costs != -1 and costs != DBL_MAX)
      if (cost_map_.at(new_edge) < 0 || cost_map_.at(new_edge) == DBL_MAX)
      {
        indices_valid = false;
        break;
      }
    }
  } while (!indices_valid);

  // swap in order to generate neighbor
  std::swap(neighbor[idx1], neighbor[idx2]);

  return neighbor;
}

std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::swapNEdges(int n)
{

  std::vector<geometry_msgs::PoseStamped> neighbor = current_path_;

  bool indices_valid;
  std::set<int> edge_end_node_idx;
  do
  {
    // randomly select edges in path to swap (by their end node index)
    // as a set is used here no check for duplicates is required
    while (edge_end_node_idx.size() < n)
    {
      int new_idx = neighbor_dis_(neighbor_gen_);

      // only add new index if (new_idx-1) and (new_idx_+1) are not in list yet
      // (with two following indices, no swap will be made as only one element would be reversed)
      if (edge_end_node_idx.find(new_idx - 1) == edge_end_node_idx.end() &&
          edge_end_node_idx.find(new_idx + 1) == edge_end_node_idx.end())
      {
        edge_end_node_idx.insert(new_idx);
      }
    }


    // reverse path parts, defined by generated edge_end_nodes
    auto start_it = neighbor.begin();
    for (auto& edge_end_idx: edge_end_node_idx)
    {
      // do not reverse first block
      if (edge_end_idx != *edge_end_node_idx.begin())
      {
        // reverse from last edge_end_idx to current edge_end_idx-1.
        auto end_it = neighbor.begin() + (edge_end_idx - 1);

        // As std::reverse(first,last) reverses [first,last), end_it needs to be incremented by one in order to be part of the reverse
        std::reverse(start_it, end_it + 1);
      }

      // update start iterator
      start_it = neighbor.begin() + edge_end_idx;
    }


    // check for each edge (simpler than first extracting the new edges as reverse might also change the costs),
    // if it is valid (costs != -1 and costs != DBL_MAX)
    indices_valid = true;
    for (int i = 1; i < neighbor.size(); ++i)
    {
      utils::PosePair edge = {neighbor[i - 1], neighbor[i]};
      if (cost_map_.at(edge) < 0 || cost_map_.at(edge) == DBL_MAX)
      {
        indices_valid = false;
        ROS_WARN_STREAM("Indices invalid.");
        neighbor = current_path_;
        break;
      }
    }
  } while (!indices_valid);

  return neighbor;
}

std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::swap2Edges()
{
  return swapNEdges(2);
}

std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::swap3Edges()
{
  return swapNEdges(3);
}


std::vector<geometry_msgs::PoseStamped> SimulatedAnnealingTspSolver::moveNode()
{
  std::vector<geometry_msgs::PoseStamped> neighbor = current_path_;

  bool indices_valid;
  int idx_to_move, idx_move_to_pos;
  do
  {
    // randomly select one node to move and one node before that the node should be moved
    idx_to_move = neighbor_dis_(neighbor_gen_);
    idx_move_to_pos = neighbor_dis_(neighbor_gen_);

    // generate new indices if positions are the same or idx to move would remain at same position
    if (idx_to_move == idx_move_to_pos || idx_to_move == (idx_move_to_pos - 1))
    {
      indices_valid = false;
      continue;
    }

    // get new edges after swap
    utils::PosePair new_edges[3] = {{neighbor[idx_move_to_pos - 1], neighbor[idx_to_move]},
                                    {neighbor[idx_to_move],         neighbor[idx_move_to_pos]},
                                    {neighbor[idx_to_move - 1],     neighbor[idx_to_move + 1]}};

    indices_valid = true;
    for (auto& new_edge: new_edges)
    {
      // check for each new edge, if it is valid (costs != -1 and costs != DBL_MAX)
      if (cost_map_.at(new_edge) < 0 || cost_map_.at(new_edge) == DBL_MAX)
      {
        indices_valid = false;
        break;
      }
    }
  } while (!indices_valid);

  // get node to move and erase it
  auto node_to_move = neighbor[idx_to_move];
  neighbor.erase(neighbor.begin() + idx_to_move);

  // insert node at new position
  neighbor.insert(neighbor.begin() + idx_move_to_pos, node_to_move);

  return neighbor;
}


double SimulatedAnnealingTspSolver::computeProbabilityForAcceptingNeighbor(double current_path_costs,
                                                                           double neighbor_path_costs)
{
  double probability;

  // "Metropolis acceptance criterion"
  probability = std::min(1.0, std::exp(-(neighbor_path_costs - current_path_costs) / temperature_));

  return probability;
}


void SimulatedAnnealingTspSolver::computeNextTemperature()
{
  // TODO evaluate to find good cooling schedule (parameter server?)
  double cooling = 0.99;
  temperature_ = cooling * temperature_;
}

void SimulatedAnnealingTspSolver::publishCurrentPath()
{
  if (!publish_path_)
  {
    return;
  }

  current_path_msg_.header.stamp = ros::Time::now();
  current_path_msg_.poses = current_path_;

  current_path_pub_.publish(current_path_msg_);
}

void SimulatedAnnealingTspSolver::writeStatisticsToFile()
{
  char buffer[256];
  auto time = static_cast<std::time_t>(ros::WallTime::now().toSec());
  auto local_time = std::localtime(&time);
  strftime(buffer, sizeof(buffer), "%y-%m-%d_%H-%M", local_time);

  // write accepted path costs
  std::string file_name = std::string(buffer) + "_simulated_annealing_accepted_costs_" +
                          std::to_string(waypoints_unordered_.size()) + "_waypoints.txt";

  file_utils::writeVectorInFile(file_name, accepted_path_costs_statistics_);


  // write all path costs
  file_name = std::string(buffer) + "_simulated_annealing_all_path_costs_" +
              std::to_string(waypoints_unordered_.size()) + "_waypoints.txt";

  file_utils::writeVectorInFile(file_name, all_path_costs_statistics_);


  // write temperatures
  file_name = std::string(buffer) + "_simulated_annealing_temperatures_" +
              std::to_string(waypoints_unordered_.size()) + "_waypoints.txt";

  file_utils::writeVectorInFile(file_name, temperature_statistics_);
}
} // end namespace hector_waypoint_order

PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::SimulatedAnnealingTspSolver,
                       hector_waypoint_order::WaypointOrderComputerBase)

