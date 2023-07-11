#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp" // Iterators

using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node {
public:
  ObstacleAvoidance() : Node("ObstacleAvoidance") {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "gazebo_ros_laser_controller/out", default_qos,
        std::bind(&ObstacleAvoidance::topic_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg) {
    float min = 10;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*_msg, "x"), iter_y(*_msg, "y"); 
         iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      float distance = std::sqrt(std::pow(*iter_x, 2) + std::pow(*iter_y, 2));
      if (distance < min) {
        min = distance;
      }
    }
    auto message = this->calculateVelMsg(min);
    publisher_->publish(message);
  }
geometry_msgs::msg::Twist calculateVelMsg(float distance) {
  static int stuck_counter = 0;  
  auto msg = geometry_msgs::msg::Twist();
  RCLCPP_INFO(this->get_logger(), "Distance is: '%f'", distance);
  if (distance < 0.2) {
    if (distance < 0.1) {
      msg.linear.x = 2; 
      msg.angular.z = 0.5;  
      stuck_counter++;
      if (stuck_counter > 1) {  
        msg.angular.z = 1;        
        msg.linear.x = 1; 
        if (stuck_counter > 2) {  
          stuck_counter = 0; 
           msg.linear.x = 2; 
           msg.angular.z = 2; 
        }
      }
    } else {
      msg.linear.x = 0;
      msg.angular.z = 1;  
      stuck_counter = 0;  
    }
  } else {
    msg.linear.x = 1;
    msg.angular.z = 0;// We're not stuck, so reset the counter
  }
  return msg;
}

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}

