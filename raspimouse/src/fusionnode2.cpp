#include "AI_API.hpp" // Include the header file with the class declarations

// Constructor implementation for BaseNode

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
