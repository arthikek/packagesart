#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

using std::placeholders::_1;

class BaseNode : public rclcpp::Node {
public:
  BaseNode(const std::string& name)
      : Node(name),
        publisher_(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)),
        lidar_subscription_(create_subscription<sensor_msgs::msg::PointCloud2>(
            "gazebo_ros_laser_controller/out",
            rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            std::bind(&BaseNode::lidar_callback, this, std::placeholders::_1))),
        camera_subscription_(create_subscription<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed",
            rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
            std::bind(&BaseNode::camera_callback, this, std::placeholders::_1))) {}

private:
  virtual void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) = 0;

  virtual void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) = 0;
  
  virtual void car_movement(const geometry_msgs::msg::Twist::SharedPtr twist_msg) = 0;
  

  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscription_;
};


int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseNode>());
  rclcpp::shutdown();
  return 0;
}
