#ifndef HECTOR_WAYPOINT_ORDER_UTILS_H
#define HECTOR_WAYPOINT_ORDER_UTILS_H

#include <map>
#include <geometry_msgs/PoseStamped.h>

namespace hector_waypoint_order
{
namespace utils
{

using PosePair = std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>;

/**
 * Elementwise comparison of lhs and rhs, returns comparison of first unequal elements.
 * If all elements are equal, 0 is returned.
 * @param lhs
 * @param rhs
 * @param precision every difference between elements of lhs and rhs < precision are counted as equal
 * @return negative value, zero, or a positive value if lhs is less than, equal to, or greater than rhs
 */
static double
compare(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs, double precision = 0)
{
  if (lhs.header.frame_id != rhs.header.frame_id)
  {
    throw std::logic_error(
      "Tried to compare two poses in different frames! lhs: " + lhs.header.frame_id + ", rhs: " + rhs.header.frame_id +
      ".");
  }

  double difference[7];
  difference[0] = lhs.pose.position.x - rhs.pose.position.x;
  difference[1] = lhs.pose.position.y - rhs.pose.position.y;
  difference[2] = lhs.pose.position.z - rhs.pose.position.z;

  difference[3] = lhs.pose.orientation.x - rhs.pose.orientation.x;
  difference[4] = lhs.pose.orientation.y - rhs.pose.orientation.y;
  difference[5] = lhs.pose.orientation.z - rhs.pose.orientation.z;
  difference[6] = lhs.pose.orientation.w - rhs.pose.orientation.w;


  for (double difference_elem: difference)
  {
    if (std::abs(difference_elem) > precision)
    {
      return difference_elem;
    }
  }
  return 0;
}


struct PoseLessComparator
{
  bool operator()(const geometry_msgs::PoseStamped& lhs, const geometry_msgs::PoseStamped& rhs) const
  {
    return compare(lhs, rhs) < 0;
  }
};


struct PosePairLessComparator
{
  bool operator()(const PosePair& lhs, const PosePair& rhs) const
  {
    // compare first element, if equal compare second element
    auto first_result = compare(lhs.first, rhs.first);
    if (first_result != 0)
    {
      return first_result < 0;
    }
    else
    {
      return compare(lhs.second, rhs.second) < 0;
    }
  }
};
} // end namespace utils

// TODO costmap: pair key or nested map?
//using CostMap = std::map<geometry_msgs::PoseStamped, std::map<geometry_msgs::PoseStamped, double, utils::PoseLessComparator>, utils::PoseLessComparator>;
using CostMap = std::map<utils::PosePair, double, utils::PosePairLessComparator>;
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_UTILS_H
