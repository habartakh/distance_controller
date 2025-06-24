#include <cmath>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class DistanceController : public rclcpp::Node {
public:
  DistanceController() : Node("distance_controller") {
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rosbot_xl_base_controller/odom", 10, std::bind(&DistanceController::topic_callback, this, _1));
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
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceController>());
  rclcpp::shutdown();
  return 0;
}