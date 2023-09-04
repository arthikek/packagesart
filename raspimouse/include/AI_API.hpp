// BaseNode.hpp

#ifndef BASE_NODE_HPP_
#define BASE_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "base_node_variables.hpp" // Include the header with topic names

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::CompressedImage> ApproximateTimePolicy;

class BaseNode : public rclcpp::Node
{
public:
  BaseNode();
  virtual ~BaseNode() = default;

private:
  virtual void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr point_cloud) = 0;
  virtual void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) = 0;
  virtual void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr &point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr &image) = 0;

protected:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscription_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> lidar_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> camera_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> ts_;

};

BaseNode::BaseNode()
    : Node("base_node"),
      publisher_(create_publisher<geometry_msgs::msg::Twist>(topic_names::CMD_VEL_TOPIC, 10)), // Use the variable from topic_names.hpp
      lidar_subscription_(create_subscription<sensor_msgs::msg::LaserScan>(
          topic_names::LIDAR_SCAN_TOPIC,
          rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
          std::bind(&BaseNode::lidar_callback, this, std::placeholders::_1))),
      camera_subscription_(create_subscription<sensor_msgs::msg::CompressedImage>(
          topic_names::CAMERA_IMAGE_TOPIC,
          rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
          std::bind(&BaseNode::camera_callback, this, std::placeholders::_1)))
{
    lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, topic_names::LIDAR_SCAN_TOPIC);
    camera_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, topic_names::CAMERA_IMAGE_TOPIC);
    ts_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(10), *lidar_sub_, *camera_sub_);
    ts_->registerCallback(&BaseNode::sensor_callback, this);
}

#endif // BASE_NODE_HPP_
