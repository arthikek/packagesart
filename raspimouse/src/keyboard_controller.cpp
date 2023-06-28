#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class KeyboardControlNode : public rclcpp::Node {
public:
  KeyboardControlNode() : Node("keyboard_control_node") {
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Set up keyboard input polling
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&KeyboardControlNode::keyboardInputCallback, this));

    RCLCPP_INFO(this->get_logger(), "Keyboard control node initialized.");
  }

private:
  void keyboardInputCallback() {
    // Read keyboard input
    int input = std::cin.get();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    // Create a new Twist message
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Set linear and angular velocities based on keyboard input
    switch (input) {
      case 'a':  // Turn left
        msg->angular.z = 1.0;
        break;

      case 'd':  // Turn right
        msg->angular.z = -1.0;
        break;

      default:
        // Stop the robot if any other key is pressed
        break;
    }

    // Publish the Twist message
    velocity_pub_->publish(*msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
