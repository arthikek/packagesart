#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp" // Iterators

using std::placeholders::_1;
constexpr double PI = 3.141592653589793;

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
 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg) {
    float min = 10;
    float angle = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*_msg, "x"), iter_y(*_msg, "y"); 
         iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      float distance = std::sqrt(std::pow(*iter_x, 2) + std::pow(*iter_y, 2));
      if (distance < min) {
        min = distance;
        angle = std::atan2(*iter_y, *iter_x);  // calculate the angle
      }
    }
 
    auto message = this->calculateVelMsg(min, angle);
    publisher_->publish(message);
  }



geometry_msgs::msg::Twist calculateVelMsg(float min_distance, float angle) {
  auto msg = geometry_msgs::msg::Twist();

  if (min_distance < 0.2) { // This is your distance threshold
    if (angle > -PI/1.5 && angle < PI/1.5) {
      msg.angular.z = 1; // Rotate if the obstacle is in front or at the sides

      // Check if the car's back is facing the wall and it's rotating slowly
      if (min_distance < 0.1 && angle > -PI && angle < PI && angle < -PI/2 && angle > PI/2 )  {
        msg.linear.x = 1; // Move briefly forward to free itself from the wall
      }
    } else {
      msg.linear.x = 1; // Move forwards if the obstacle is at the back
    }
  } else {
    msg.linear.x = 1; // Move forward
    msg.angular.z = 0; // Stop turning
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

