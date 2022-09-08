#ifndef _AZURE_KINECT_ROS2__AZURE_KINECT_NODE_HPP_
#define _AZURE_KINECT_ROS2__AZURE_KINECT_NODE_HPP_

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <k4a/k4a.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace azure_kinect_ros2
{
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Image = sensor_msgs::msg::Image;

struct k4a_int16_vec_t
{
  int16_t x;
  int16_t y;
  int16_t z;
};

struct k4a_bgra_t
{
  uint8_t b;
  uint8_t g;
  uint8_t r;
  uint8_t a;
};

struct k4a_rgb_t
{
  float r;
  float g;
  float b;
};

struct k4a_rgbd_t
{
  k4a_float3_t xyz;
  k4a_rgb_t rgb;
};

class AzureKinectNode : public rclcpp::Node
{
  bool open_with_launch_ = false;
  bool start_with_launch_ = false;
  bool device_is_opened_ = false;
  bool device_is_started_ = false;
  int fps_ = 30;
  bool publish_3d_ = false;
  std::string frame_id_ = "azure_kinect";

  std::unique_ptr<k4a::device> device_;
  std::unique_ptr<k4a::transformation> transformation_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pc2_pub_;
  rclcpp::Publisher<Image>::SharedPtr depth_image_pub_;
  rclcpp::Publisher<Image>::SharedPtr color_image_pub_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  k4a_device_configuration_t device_config_;

  k4a_depth_mode_t depth_mode_from_param();
  k4a_image_format_t image_format_from_param();
  k4a_fps_t fps_from_param();
  k4a_color_resolution_t color_resolution_from_param();
public:
  AzureKinectNode();
  void open_device();
  void start_device();
  void capture_loop();
  void publish_pointcloud(const k4a::image& depth, const k4a::image& color);
  void publish_images(const k4a::image& depth, const k4a::image& color);
};
}

#endif
