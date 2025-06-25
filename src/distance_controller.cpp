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
  DistanceController() : Node("distance_controller") {

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10,
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

    dx = current_x - old_x;
    dy = current_y - old_y;

    distance_travelled_x += dx;
    distance_travelled_y += dy;

    distance_travelled += std::sqrt(dx * dx + dy * dy);

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

    if (traj_index >= waypoints_traj.size()) {
      stop_robot();
      RCLCPP_INFO_ONCE(this->get_logger(), "Completed the trajectory! ");
      rclcpp::shutdown();
    }

    WayPoint target = waypoints_traj[traj_index];

    // Compute the distance left to the target
    double error_x = target.dx - distance_travelled_x;
    double error_y = target.dy - distance_travelled_y;
    double distance = std::hypot(error_x, error_y);

    // If the robot reached the target waypoint
    if (distance < 0.02) {
      RCLCPP_INFO(this->get_logger(), "Reached waypoint number : %lu",
                  traj_index + 1);

      // Stop the robot for 20 * 0.1 = 2 seconds

      stop_robot();

      traj_index++; // Update the next motion index

      // reset distances travelled to compute next trajectory errors
      distance_travelled_x = 0.0;
      distance_travelled_y = 0.0;
      rclcpp::sleep_for(std::chrono::seconds(2));
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
  long unsigned int traj_index = 0;

  // PID controller parameters
  // NOTE: Bigger Kd values result in oscillations in the direction x/y when
  // dx/dy = 0 That is because even though the error following these axes is
  // close to 0, multiplying it by Kd and adding it to vx/vy gives unwanted
  // velocity to these axes Thus making the robot oscillate uncontrollably
  double kp = 3.5;         // Proportional Gain
  double ki = 0.05;        // Integral Gain
  double kd = 2.0;         // Derivative Gain
  double integral_x = 0.0; // Integral terms of the PID controller
  double integral_y = 0.0;
  rclcpp::Time prev_time; // instant t-1
  double prev_error_x = 0.0;
  double prev_error_y = 0.0;
  double max_speed = 0.8; // source: https://husarion.com/manuals/rosbot-xl/

  // Parameters to move the robot
  geometry_msgs::msg::Twist twist_cmd;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<DistanceController> distance_controller =
      std::make_shared<DistanceController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(distance_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}