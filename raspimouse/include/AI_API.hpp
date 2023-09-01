#ifndef BASE_NODE_HPP_
#define BASE_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage> ApproximateTimePolicy;

class BaseNode : public rclcpp::Node
{
public:
  BaseNode();
  virtual ~BaseNode() = default;

private:
  virtual void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) = 0;
  virtual void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) = 0;
  virtual void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr &image) = 0;

protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscription_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> camera_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> ts_;

};

BaseNode::BaseNode()
    : Node("base_node"),
      publisher_(create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10)),
      lidar_subscription_(create_subscription<sensor_msgs::msg::PointCloud2>(
          "gazebo_ros_laser_controller/out",
          rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
          std::bind(&BaseNode::lidar_callback, this, std::placeholders::_1))),
      camera_subscription_(create_subscription<sensor_msgs::msg::CompressedImage>(
          "/camera/image_raw/compressed",
          rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
          std::bind(&BaseNode::camera_callback, this, std::placeholders::_1)))
{
    lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "gazebo_ros_laser_controller/out");
    camera_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, "/camera/image_raw/compressed");
    ts_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(10), *lidar_sub_, *camera_sub_);
    ts_->registerCallback(&BaseNode::sensor_callback, this);
}

#endif // BASE_NODE_HPP_
