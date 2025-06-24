#include <cmath>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

struct WayPoint {
  double dx;   // change in the x-coordinate of the robot's position
  double dy;   // change in the y-coordinate of the robot's position
  double dphi; // change in the orientation angle of the robot

  WayPoint(double a, double b, double c = 0.0) : dx(a), dy(b), dphi(c) {}
};

enum MotionType { // To be executed by the rosbot
  WEST,
  WEST_REVERSE,
  EAST,
  EAST_REVERSE,
  NORTH_WEST,
  NORTH_WEST_REVERSE,
  NORTH_EAST,
  NORTH_EAST_REVERSE,
  NORTH,
  NORTH_REVERSE
};

// Overload the ++ operator for enum to increment the motion types
// source: https://cplusplus.com/forum/beginner/41790/
inline MotionType &operator++(MotionType &eDOW, int) {
  const int i = static_cast<int>(eDOW);
  eDOW = static_cast<MotionType>((i + 1) % 10);
  return eDOW;
}

class DistanceController : public rclcpp::Node {
public:
  DistanceController() : Node("distance_controller") {
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
        std::bind(&DistanceController::topic_callback, this, _1));

    // Initialize the trajectory waypoints
    waypoints_traj_init();
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Compute distance travelled in X and Y axis
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    dx = current_x - old_x;
    dy = current_y - old_y;

    distance_travelled_x += dx;
    distance_travelled_y += dy;

    distance_travelled += std::sqrt(dx * dx + dy * dy);

    RCLCPP_INFO(this->get_logger(), "dx = %f ", dx);
    RCLCPP_INFO(this->get_logger(), "dy = %f ", dy);
    RCLCPP_INFO(this->get_logger(), "distance_travelled_x = %f ",
                distance_travelled_x);
    RCLCPP_INFO(this->get_logger(), "distance_travelled_y = %f ",
                distance_travelled_y);

    old_x = current_x;
    old_y = current_y;
  }

  // Add all the waypoints the robot is going throughout the trajectory
  void waypoints_traj_init() {

    waypoints_traj.push_back(WayPoint(+0.000, +1.000)); // Step 1
    waypoints_traj.push_back(WayPoint(+0.000, -1.000)); // Step 2
    waypoints_traj.push_back(WayPoint(+0.000, -1.000)); // Step 3
    waypoints_traj.push_back(WayPoint(+0.000, +1.000)); // 4
    waypoints_traj.push_back(WayPoint(+1.000, +1.000)); // 5
    waypoints_traj.push_back(WayPoint(-1.000, -1.000)); // 6
    waypoints_traj.push_back(WayPoint(+1.000, -1.000)); // 7
    waypoints_traj.push_back(WayPoint(-1.000, +1.000)); // 8
    waypoints_traj.push_back(WayPoint(+1.000, +0.000)); // 9
    waypoints_traj.push_back(WayPoint(-1.000, +0.000)); // 10
  }

  // Move the robot according to the desired trajectory
  void control_loop() {

    switch (motion_type) {

    case WEST:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move WEST");
      break;

    case WEST_REVERSE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move WEST_REVERSE");
      break;

    case EAST:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move EAST");
      break;

    case EAST_REVERSE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move EAST_REVERSE");
      break;

    case NORTH_WEST:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH_WEST");
      break;

    case NORTH_WEST_REVERSE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH_WEST_REVERSE");
      break;

    case NORTH_EAST:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH_EAST");
      break;

    case NORTH_EAST_REVERSE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH_EAST_REVERSE");
      break;

    case NORTH:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH");
      break;

    case NORTH_REVERSE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Move NORTH_REVERSE");
      break;
    }
  }

  // Variable declarations
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  // Parameters used to compute the distance travelled
  double old_x = 0.0;
  double old_y = 0.0;
  double current_x = 0.0;
  double current_y = 0.0;
  double dx = 0.0; // distance between two odom messages
  double dy = 0.0;
  double distance_travelled_x = 0.0;
  double distance_travelled_y = 0.0;
  double distance_travelled = 0.0;

  // Waypoints the robot is passing by
  std::vector<WayPoint> waypoints_traj;
  // The direction that should be followed by the rosbot
  enum MotionType motion_type;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}