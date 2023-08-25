#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
// "message_filters/subscriber.h" includes the definition of the "Subscriber" class
// from the "message_filters" library. This class provides advanced topic
// subscription functionalities, like synchronizing messages from different topics.
#include "message_filters/subscriber.h"

// "message_filters/sync_policies/approximate_time.h" includes the definition
// of the "ApproximateTime" synchronization policy. This policy is used to
// synchronize messages from different topics that are not perfectly 
// synchronized, but have close enough timestamps.
#include "message_filters/sync_policies/approximate_time.h"

// Define a type alias "ApproximateTimePolicy" for an ApproximateTime 
// synchronization policy that works with PointCloud2 and CompressedImage 
// messages. It will be used to ensure that for each pair of PointCloud2 and 
// CompressedImage messages processed, their timestamps are as close as 
// possible, even if they aren't exactly the same. This is useful when you want
// to process data from different sensors that may not be perfectly synchronized.
typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::CompressedImage> ApproximateTimePolicy;

class BaseNode : public rclcpp::Node {
public:
  BaseNode()
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
    // Initialize subscribers for message_filters
    lidar_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "gazebo_ros_laser_controller/out");
    camera_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(this, "/camera/image_raw/compressed");

    // Initialize Approximate Time Synchronizer
   
    ts_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(ApproximateTimePolicy(10), *lidar_sub_, *camera_sub_);

    ts_->registerCallback(&BaseNode::sensor_callback, this);
  }

private:
  virtual void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) =0;

  virtual void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) = 0;

  virtual void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr& image) = 0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscription_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> lidar_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> camera_sub_;
   std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> ts_;
};

class DerivedNode : public BaseNode {
public:
  DerivedNode() : BaseNode() {}

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) override {
  
    
    uint32_t num_points = point_cloud->width * point_cloud->height;
    

    if(num_points > 0) {
      sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");
     
    }
  }

  void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) override {
    
    
    auto format = image->format;  // MIME type
    auto size = image->data.size();  // Size in bytes


  }

void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr& image) override {
    RCLCPP_INFO(this->get_logger(), "Received synchronized lidar and image data");

    uint32_t num_points = point_cloud->width * point_cloud->height;
    RCLCPP_INFO(this->get_logger(), "Number of points in the point cloud: %u", num_points);

    auto format = image->format;  // MIME type
    auto size = image->data.size();  // Size in bytes
    RCLCPP_INFO(this->get_logger(), "Image format: %s, size: %lu bytes", format.c_str(), size);

    // Calculate the time difference between lidar and camera messages
    rclcpp::Time lidar_time = point_cloud->header.stamp;
    rclcpp::Time camera_time = image->header.stamp;
    rclcpp::Duration time_diff = lidar_time - camera_time;

    // Print out the time difference
    RCLCPP_INFO(this->get_logger(), "Time difference between lidar and image data: %f seconds", time_diff.seconds());
}

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DerivedNode>());
  rclcpp::shutdown();
  return 0;
}
