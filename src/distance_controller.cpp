#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

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

    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    twist_timer = this->create_wall_timer(
        100ms, std::bind(&DistanceController::control_loop, this));

    // Initialize the trajectory waypoints
    waypoints_traj_init();

    RCLCPP_INFO(this->get_logger(), "Initialized distance controller node");
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

    // RCLCPP_INFO(this->get_logger(), "dx = %f ", dx);
    // RCLCPP_INFO(this->get_logger(), "dy = %f ", dy);
    // RCLCPP_INFO(this->get_logger(), "distance_travelled_x = %f ",
    //          distance_travelled_x);
    // RCLCPP_INFO(this->get_logger(), "distance_travelled_y = %f ",
    //           distance_travelled_y);

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

    // First test only on one motion type

    WayPoint target = waypoints_traj[0]; // West direction

    // Compute the distance left to the target
    double error_x = target.dx - distance_travelled_x;
    double error_y = target.dy - distance_travelled_y;
    double distance = std::hypot(error_x, error_y);
    /*
        RCLCPP_INFO(this->get_logger(), "target.dx = %f ", target.dx);
        RCLCPP_INFO(this->get_logger(), "target.dy = %f ", target.dy);
        RCLCPP_INFO(this->get_logger(), "error_x = %f ", error_x);
        RCLCPP_INFO(this->get_logger(), "error_y = %f ", error_y);
        RCLCPP_INFO(this->get_logger(), "distance = %f ", distance);
    */
    // stop after reaching the target
    if (distance < 0.02) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint!! ");
      stop_robot();
      return;
    }

    // Calculate delta time
    rclcpp::Time now = this->now();
    double dt =
        prev_time.nanoseconds() == 0 ? 0.05 : (now - prev_time).seconds();
    prev_time = now;

    // Integral terms
    integral_x += error_x * dt;
    integral_y += error_y * dt;

    RCLCPP_INFO(this->get_logger(), "integral_x = %f ", integral_x);
    RCLCPP_INFO(this->get_logger(), "integral_y = %f ", integral_y);

    // Derivative terms
    double derivative_x = (error_x - prev_error_x) / dt;
    double derivative_y = (error_y - prev_error_y) / dt;

    RCLCPP_INFO(this->get_logger(), "derivative_x = %f ", derivative_x);
    RCLCPP_INFO(this->get_logger(), "derivative_y = %f ", derivative_y);

    // PID control signal
    double vx = kp * error_x + ki * integral_x + kd * derivative_x;
    double vy = kp * error_y + ki * integral_y + kd * derivative_y;

    RCLCPP_INFO(this->get_logger(), "vx = %f ", vx);
    RCLCPP_INFO(this->get_logger(), "vy = %f ", vy);

    twist_cmd.linear.x = vx;
    twist_cmd.linear.y = vy;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(twist_cmd);

    prev_error_x = error_x;
    prev_error_y = error_y;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist stop_msg;
    twist_cmd.linear.x = 0.0;
    twist_cmd.linear.y = 0.0;
    twist_cmd.angular.z = 0.0;

    twist_pub->publish(stop_msg);
  }

  // Variable declarations
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

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

  // PID controller parameters
  double kp = 1.5;         // Proportional Gain
  double ki = 0.0;         // Integral Gain
  double kd = 0.3;         // Derivative Gain
  double integral_x = 0.0; // Integral terms of the PID controller
  double integral_y = 0.0;
  rclcpp::Time prev_time; // instant t-1
  double prev_error_x = 0.0;
  double prev_error_y = 0.0;

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}