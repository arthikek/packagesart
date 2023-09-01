#include "AI_API.hpp" // Include the header file with the class declarations

// Constructor implementation for BaseNode

// DerivedNode class that inherits from BaseNode
class DerivedNode : public BaseNode
{
public:
    DerivedNode() : BaseNode() {}

    // Existing implementation for lidar_callback
void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) override
{
    RCLCPP_INFO(this->get_logger(), "Lidar callback triggered");

    float min_distance = std::numeric_limits<float>::max();
    RCLCPP_DEBUG(this->get_logger(), "Initialized min_distance to max float value: %f", min_distance);

    // Loop through scan ranges and find the minimum distance.
    for(auto distance : scan->ranges)
    {
        if (distance < min_distance)
        {
            RCLCPP_DEBUG(this->get_logger(), "Found a new min distance: %f", distance);
            min_distance = distance;
        }
    }

        // Publish a twist message based on the LaserScan data.
        auto twist_msg = geometry_msgs::msg::Twist();
        
        if (min_distance < 0.2) // if obstacle is closer than 0.5m
        {
            // Stop the robot
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
        }
        else
        {
            // Move the robot forward
            twist_msg.linear.x = 0.2;
            twist_msg.angular.z = 0.0;
        }

    // Publish a twist message based on the LaserScan data.
    auto twist_msg = geometry_msgs::msg::Twist();
    
    if (min_distance < 0.5) // if obstacle is closer than 0.5m
    {
        RCLCPP_WARN(this->get_logger(), "Obstacle too close! Stopping the robot.");
        // Stop the robot
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Path is clear. Moving the robot forward.");
        // Move the robot forward
        twist_msg.linear.x = 0.5;
        twist_msg.angular.z = 0.0;
    }

    publisher_->publish(twist_msg);
    RCLCPP_INFO(this->get_logger(), "Published Twist message to /cmd_vel");
}

    // Existing implementation for camera_callback
    void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image) override
    {

        auto format = image->format;    // MIME type
        auto size = image->data.size(); // Size in bytes
    }
    // Existing implementation for sensor_callback
    void sensor_callback(const sensor_msgs::msg::LaserScan::SharedPtr &point_cloud, const sensor_msgs::msg::CompressedImage::SharedPtr &image) override
    {
        RCLCPP_INFO(this->get_logger(), "Received synchronized lidar and image data");

        uint32_t num_points = point_cloud->ranges.size();
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
