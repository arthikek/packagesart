#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

class BaseNode : public rclcpp::Node {
public:
// implement base class pointers to connect to subscriber and publisher
  BaseNode(const std::string& name)
      : Node(name),
        publisher_(this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)),
        subscription_(this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "gazebo_ros_laser_controller/out", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            std::bind(&BaseNode::topic_callback, this, _1))) {}

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr _msg) {
    float min = 10;
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*_msg, "x"),
                                                     iter_y(*_msg, "y");
         iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      float distance = std::sqrt(std::pow(*iter_x, 2) + std::pow(*iter_y, 2));
      if (distance < min) {
        min = distance;
      }
    }
    auto message = this->calculateVelMsg(min);
    publisher_->publish(message);
  }

  virtual geometry_msgs::msg::Twist calculateVelMsg(float distance) = 0;
protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

class ObstacleAvoidance : public BaseNode {
public:
  ObstacleAvoidance() : BaseNode("ObstacleAvoidance") {}

private:
  geometry_msgs::msg::Twist calculateVelMsg(float distance) override {
    
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidance>());
  rclcpp::shutdown();
  return 0;
}
