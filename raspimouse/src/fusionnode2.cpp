#include "AI_API.hpp" // Include the header file with the class declarations

// Constructor implementation for BaseNode
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

// DerivedNode class that inherits from BaseNode
class DerivedNode : public BaseNode
{
public:
    DerivedNode() : BaseNode() {}

    // Existing implementation for lidar_callback
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) override
    {

        uint32_t num_points = point_cloud->width * point_cloud->height;

        if (num_points > 0)
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");
        }
    }

    // Existing implementation for camera_callback
    void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) override
    {

        auto format = image->format;    // MIME type
        auto size = image->data.size(); // Size in bytes
    }
    // Existing implementation for sensor_callback
    void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr &point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr &image) override
    {
        RCLCPP_INFO(this->get_logger(), "Received synchronized lidar and image data");

        uint32_t num_points = point_cloud->width * point_cloud->height;
        RCLCPP_INFO(this->get_logger(), "Number of points in the point cloud: %u", num_points);

        auto format = image->format;    // MIME type
        auto size = image->data.size(); // Size in bytes
        RCLCPP_INFO(this->get_logger(), "Image format: %s, size: %lu bytes", format.c_str(), size);

        // Calculate the time difference between lidar and camera messages
        rclcpp::Time lidar_time = point_cloud->header.stamp;
        rclcpp::Time camera_time = image->header.stamp;
        rclcpp::Duration time_diff = lidar_time - camera_time;

        // Print out the time difference
        RCLCPP_INFO(this->get_logger(), "Time difference between lidar and image data: %f seconds", time_diff.seconds());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DerivedNode>());
    rclcpp::shutdown();
    return 0;
}
