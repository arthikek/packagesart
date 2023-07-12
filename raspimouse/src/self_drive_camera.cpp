#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using std::placeholders::_1;

class ImageAnalyzer : public rclcpp::Node
{
public:
  ImageAnalyzer() : Node("ImageAnalyzer")
  {
    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/camera/image_raw/compressed", default_qos,
        std::bind(&ImageAnalyzer::image_callback, this, _1));

    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&ImageAnalyzer::log_color, this));
  }

private:
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    // Convert the compressed image message to a cv::Mat.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Compute the average color of the image.
    avg_color_ = cv::mean(cv_ptr->image);
  }

void log_color()
{
  // Define named colors.
  std::map<std::string, cv::Scalar> named_colors = {
    {"Black", cv::Scalar(0, 0, 0)},
    {"White", cv::Scalar(255, 255, 255)},
    {"Red", cv::Scalar(0, 0, 255)},
    {"Lime", cv::Scalar(0, 255, 0)},
    {"Blue", cv::Scalar(255, 0, 0)},
    {"Yellow", cv::Scalar(0, 255, 255)},
    {"Cyan", cv::Scalar(255, 255, 0)},
    {"Magenta", cv::Scalar(255, 0, 255)},
    {"Silver", cv::Scalar(192, 192, 192)},
    {"Gray", cv::Scalar(128, 128, 128)},
    {"Maroon", cv::Scalar(0, 0, 128)},
    {"Olive", cv::Scalar(0, 128, 128)},
    {"Green", cv::Scalar(0, 128, 0)},
    {"Purple", cv::Scalar(128, 0, 128)},
    {"Teal", cv::Scalar(128, 128, 0)},
    {"Navy", cv::Scalar(128, 0, 0)}
  };

  // Find the closest named color.
  double min_distance = std::numeric_limits<double>::max();
  std::string closest_color;
  for (const auto& named_color : named_colors)
  {
    double distance = cv::norm(avg_color_ - named_color.second);
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_color = named_color.first;
    }
  }

  // Log the average color and its name.
  RCLCPP_INFO(this->get_logger(), "Average color: B=%f, G=%f, R=%f (%s)",
              avg_color_[0], avg_color_[1], avg_color_[2], closest_color.c_str());
}

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Scalar avg_color_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageAnalyzer>());
  rclcpp::shutdown();
  return 0;
}
