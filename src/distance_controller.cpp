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

class DistanceController : public rclcpp::Node {
public:
  DistanceController(int scene_number)
      : Node("distance_controller"), scene_number_(scene_number) {

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&DistanceController::odom_callback, this, _1), options1);

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    twist_timer = this->create_wall_timer(
        100ms, std::bind(&DistanceController::control_loop, this),
        timer_callback_group_);

    // Initialize the trajectory waypoints
    waypoints_traj_init();

    RCLCPP_INFO(this->get_logger(), "Initialized distance controller node");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Compute distance travelled in X and Y axis
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    // To only move the robot when valid messages are received
    odom_received = true;
  }

  // Add all the waypoints the robot is going throughout the trajectory
  void waypoints_traj_init() {

    switch (scene_number_) {

    case 1: // Simulation
      // Assign waypoints for Simulation
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
      break;

    case 2: // CyberWorld
      // Assign waypoints for CyberWorld
      waypoints_traj.push_back(WayPoint(+0.930, +0.000)); // Step 1
      waypoints_traj.push_back(WayPoint(+0.000, -0.543)); // Step 2
      waypoints_traj.push_back(WayPoint(+0.000, +0.543)); // Step 3
      waypoints_traj.push_back(WayPoint(-0.850, +0.000)); // Step 4
      max_speed = 0.2; // Decrease robot speed to prevent damage
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d",
                   scene_number_);
    }
  }

  // Move the robot according to the desired trajectory
  void control_loop() {
    if (traj_index >= waypoints_traj.size()) {
      stop_robot();
      RCLCPP_INFO_ONCE(this->get_logger(), "Completed the trajectory! ");
      rclcpp::shutdown();
    }

    if (!odom_received) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for odometry...");
      return;
    }

    // If the robot is stopped between motions
    if (is_stopping) {
      stop_robot();
      stop_iterations++;

      if (stop_iterations >= 20) {
        RCLCPP_INFO(this->get_logger(), "Reached waypoint number : %lu",
                    traj_index + 1);

        traj_index++; // Move to next waypoint
        move_initialized = false;
        integral_x = 0.0;
        integral_y = 0.0;

        stop_iterations = 0;
        is_stopping = false;
      }
      return; // ← Important! Skip the rest of the loop
    }

    WayPoint target = waypoints_traj[traj_index];

    // Compute the yaw at the start of the movement
    if (!move_initialized) {
      x_start = current_x;
      y_start = current_y;
      move_initialized = true;
    }

    double goal_x = x_start + target.dx;
    double goal_y = y_start + target.dy;

    // Compute the distance left to the target
    double error_x = goal_x - current_x;
    double error_y = goal_y - current_y;

    // When dx or dy = 0 during lateral movements,
    // the corresponding error still accumulates noise, resulting in
    // non null velocities that need to be corrected
    if (target.dx != 0.00 && target.dy == 0.000) {
      error_y = 0.0;
    }
    if (target.dy != 0.00 && target.dx == 0.000) {
      error_x = 0.0;
    }

    double distance = std::hypot(error_x, error_y);

    RCLCPP_INFO(this->get_logger(), "error_x = %f ", error_x);
    RCLCPP_INFO(this->get_logger(), "error_y = %f ", error_y);

    // If the robot reached the target waypoint
    if (distance < 0.03) {

      is_stopping = true; // ← Trigger stop phase
      stop_iterations = 0;
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

    // Derivative terms
    double derivative_x = (error_x - prev_error_x) / dt;
    double derivative_y = (error_y - prev_error_y) / dt;

    // PID control law
    double vx = kp * error_x + ki * integral_x + kd * derivative_x;
    double vy = kp * error_y + ki * integral_y + kd * derivative_y;

    // Make sure the robot stays within max speed bounds
    // std::cout << "max_speed : " << max_speed << std::endl;
    vx = std::clamp(vx, -max_speed, +max_speed);
    vy = std::clamp(vy, -max_speed, +max_speed);

    // RCLCPP_INFO(this->get_logger(), "vx = %f ", vx);
    // RCLCPP_INFO(this->get_logger(), "vy = %f ", vy);

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
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  // Parameters used to compute the distance travelled
  double current_x = 0.0;
  double current_y = 0.0;
  double x_start = 0.0;
  double y_start = 0.0;
  bool move_initialized = false;
  bool odom_received = false;
  int stop_iterations = 0;
  bool is_stopping = false;

  // Waypoints the robot is passing by
  std::vector<WayPoint> waypoints_traj;
  long unsigned int traj_index = 0;

  // PID controller parameters
  // NOTE: Bigger Kd values result in oscillations in the direction x/y when
  // dx/dy = 0. That is because even though the error following these axes is
  // close to 0, multiplying it by Kd and adding it to vx/vy gives unwanted
  // velocity to these axes. Thus making the robot oscillate uncontrollably
  double kp = 1.0;         // Proportional Gain
  double ki = 0.05;        // Integral Gain
  double kd = 0.0;         // Derivative Gain
  double integral_x = 0.0; // Integral terms of the PID controller
  double integral_y = 0.0;
  rclcpp::Time prev_time; // instant t-1
  double prev_error_x = 0.0;
  double prev_error_y = 0.0;
  double max_speed = 0.8; // source: https://husarion.com/manuals/rosbot-xl/

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;

  // Distinguish between simulation and real scenarios
  int scene_number_; // 1 : Sim; 2: Real
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Check if a scene number argument is provided
  int scene_number = 1; // Default scene number to simulation
  if (argc > 1) {
    scene_number = std::atoi(argv[1]);
  }

  std::shared_ptr<DistanceController> distance_controller =
      std::make_shared<DistanceController>(scene_number);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(distance_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}