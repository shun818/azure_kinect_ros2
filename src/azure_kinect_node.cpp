#include "azure_kinect_ros2/azure_kinect_node.hpp"
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

const std::map<std::string, k4a_depth_mode_t> depth_mode_str_map = {
  {"OFF", K4A_DEPTH_MODE_OFF},
  {"NFOV_2X2BINNED", K4A_DEPTH_MODE_NFOV_2X2BINNED},
  {"NFOV_UNBINNED", K4A_DEPTH_MODE_NFOV_UNBINNED},
  {"WFOV_2X2BINNED", K4A_DEPTH_MODE_WFOV_2X2BINNED},
  {"WFOV_UNBINNED", K4A_DEPTH_MODE_WFOV_UNBINNED},
  {"PASSIVE_IR", K4A_DEPTH_MODE_PASSIVE_IR}
};

const std::map<std::string, k4a_image_format_t> image_format_str_map = {
  {"COLOR_MJPG", K4A_IMAGE_FORMAT_COLOR_MJPG},
  {"COLOR_NV12", K4A_IMAGE_FORMAT_COLOR_NV12},
  {"COLOR_YUY2", K4A_IMAGE_FORMAT_COLOR_YUY2},
  {"COLOR_BGRA32", K4A_IMAGE_FORMAT_COLOR_BGRA32},
  {"DEPTH16", K4A_IMAGE_FORMAT_DEPTH16},
  {"IR16", K4A_IMAGE_FORMAT_IR16},
  {"CUSTOM8", K4A_IMAGE_FORMAT_CUSTOM8},
  {"CUSTOM16", K4A_IMAGE_FORMAT_CUSTOM16},
  {"CUSTOM", K4A_IMAGE_FORMAT_CUSTOM}
};

const std::map<int, k4a_color_resolution_t> color_resolution_value_map = {
  {0, K4A_COLOR_RESOLUTION_OFF},
  {720, K4A_COLOR_RESOLUTION_720P},
  {1080, K4A_COLOR_RESOLUTION_1440P},
  {1536, K4A_COLOR_RESOLUTION_1536P},
  {2160, K4A_COLOR_RESOLUTION_2160P},
  {3072, K4A_COLOR_RESOLUTION_3072P}
};

const std::map<int, k4a_fps_t> fps_value_map = {
  {5, K4A_FRAMES_PER_SECOND_5},
  {15, K4A_FRAMES_PER_SECOND_15},
  {30, K4A_FRAMES_PER_SECOND_30}
};

namespace azure_kinect_ros2
{
AzureKinectNode::AzureKinectNode()
  : Node("azure_kinect_node")
{
  declare_parameter<bool>("open_with_launch", true);
  declare_parameter<bool>("start_with_launch", true);
  declare_parameter<int>("device_id", K4A_DEVICE_DEFAULT);
  declare_parameter<std::string>("depth_mode", "NFOV_2X2BINNED");
  declare_parameter<std::string>("image_format", "COLOR_BGRA32");
  declare_parameter<bool>("publish_3d", false);
  declare_parameter<int>("fps", 30);
  declare_parameter<int>("color_resolution", 1080);

  bool open_with_launch, start_with_launch;
  get_parameter("open_with_launch", open_with_launch);
  get_parameter("start_with_launch", start_with_launch);
  get_parameter("publish_3d", publish_3d_);
  pc2_pub_ = create_publisher<PointCloud2>("~/point_cloud2", 3);
  color_image_pub_ = create_publisher<Image>("~/color", 1);
  depth_image_pub_ = create_publisher<Image>("~/depth", 1);
  if (open_with_launch) {
    RCLCPP_INFO(get_logger(), "Open device");
    open_device();
  }
  if (device_is_opened_ && start_with_launch) {
    RCLCPP_INFO(get_logger(), "Start device");
    start_device();
  }
}

k4a_depth_mode_t AzureKinectNode::depth_mode_from_param()
{
  std::string value_str;
  get_parameter("depth_mode", value_str);
  return depth_mode_str_map.at(value_str);
}

k4a_image_format_t AzureKinectNode::image_format_from_param()
{
  std::string value_str;
  get_parameter("image_format", value_str);
  return image_format_str_map.at(value_str);
}

k4a_fps_t AzureKinectNode::fps_from_param()
{
  int value;
  get_parameter("fps", value);
  fps_ = value;
  return fps_value_map.at(value);
}

k4a_color_resolution_t AzureKinectNode::color_resolution_from_param()
{
  int value;
  get_parameter("color_resolution", value);
  return color_resolution_value_map.at(value);
}

void AzureKinectNode::open_device()
{
  int device_id;
  get_parameter("device_id", device_id);
  try {
    device_ = std::make_unique<k4a::device>(k4a::device::open(device_id));
    device_is_opened_ = true;
  }
  catch (k4a::error& err) {
    RCLCPP_ERROR(get_logger(), "%s", err.what());
  }
}

void AzureKinectNode::start_device()
{
  if (!device_is_opened_) {
    RCLCPP_ERROR(get_logger(), "Device not opened.");
    return;
  }
  device_config_ = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  device_config_.depth_mode = depth_mode_from_param();
  device_config_.camera_fps = fps_from_param();
  auto color_resolution = color_resolution_from_param();
  if (color_resolution != K4A_COLOR_RESOLUTION_OFF) {
    device_config_.color_format = image_format_from_param();
    device_config_.color_resolution = color_resolution;
  }

  try {
    device_->start_cameras(&device_config_);
    transformation_ = std::make_unique<k4a::transformation>(
      device_->get_calibration(device_config_.depth_mode, device_config_.color_resolution));
    device_is_started_ = true;
    loop_timer_ = create_wall_timer(std::chrono::milliseconds((int)(1000.0 / fps_)),
      std::bind(&AzureKinectNode::capture_loop, this));
  }
  catch (k4a::error& err) {
    RCLCPP_ERROR(get_logger(), "%s", err.what());
  }
  RCLCPP_INFO(get_logger(), "Capture starting");
}

void AzureKinectNode::capture_loop()
{
  k4a::capture cap;
  if (!device_->get_capture(&cap, std::chrono::milliseconds((int)(1000 / fps_))))
    return;
  k4a::image depth_image = cap.get_depth_image();
  k4a::image color_image = cap.get_color_image();
  if (!depth_image || !color_image)
    return;
  
  if (publish_3d_) {
    publish_pointcloud(depth_image, color_image);
  }
  else {
    publish_images(depth_image, color_image);
  }
}

void AzureKinectNode::publish_pointcloud(const k4a::image& depth, const k4a::image& color)
{
  PointCloud2 msg;
  // create header
  msg.header.frame_id = frame_id_;
  msg.header.stamp = this->now();
  std::array<std::string, 6> field_names = {"x", "y", "z", "r", "g", "b"};
  for (size_t i = 0; i < field_names.size(); ++i) {
    sensor_msgs::msg::PointField field;
    field.name = field_names[i];
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.offset = sizeof(float) * i;
    field.count = 1;
    msg.fields.push_back(field);
  }

  k4a::image transformed_depth_image = transformation_->depth_image_to_color_camera(depth);
  k4a::image xyz_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
    transformed_depth_image.get_width_pixels(),
    transformed_depth_image.get_height_pixels(),
    transformed_depth_image.get_stride_bytes() * 3);
  transformation_->depth_image_to_point_cloud(
    transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &xyz_image);
  size_t points_num = transformed_depth_image.get_width_pixels() * transformed_depth_image.get_height_pixels();
  msg.height = transformed_depth_image.get_height_pixels();
  msg.width = transformed_depth_image.get_width_pixels();
  msg.point_step = sizeof(k4a_rgbd_t);
  msg.row_step = sizeof(k4a_rgbd_t) * transformed_depth_image.get_width_pixels();

  msg.data.resize(sizeof(k4a_rgbd_t)*points_num);
  k4a_rgbd_t* dst_ptr = (k4a_rgbd_t*)msg.data.data();
  k4a_int16_vec_t* xyz_ptr = (k4a_int16_vec_t*)xyz_image.get_buffer();
  k4a_bgra_t* color_ptr = (k4a_bgra_t*)color.get_buffer();
  for (size_t i = 0; i < points_num; ++i) {
    dst_ptr[i].xyz.xyz.x = (float)xyz_ptr[i].x / 1000;
    dst_ptr[i].xyz.xyz.y = (float)xyz_ptr[i].y / 1000;
    dst_ptr[i].xyz.xyz.z = (float)xyz_ptr[i].z / 1000;
    dst_ptr[i].rgb.r = (float)color_ptr[i].r / 255.0;
    dst_ptr[i].rgb.g = (float)color_ptr[i].g / 255.0;
    dst_ptr[i].rgb.b = (float)color_ptr[i].b / 255.0;
  }
  pc2_pub_->publish(msg);
}

void AzureKinectNode::publish_images(const k4a::image& depth, const k4a::image& color)
{
  std_msgs::msg::Header color_header;
  color_header.frame_id = frame_id_;
  color_header.stamp = now();
  sensor_msgs::msg::Image color_msg;
  color_msg.width = color.get_width_pixels();
  color_msg.height = color.get_height_pixels();
  color_msg.encoding = "bgra8";
  color_msg.is_bigendian = 1;
  color_msg.step = sizeof(uint8_t)*4*color.get_width_pixels();
  color_msg.data.resize(color.get_size());
  std::memcpy(color_msg.data.data(), color.get_buffer(), color.get_size());
  color_image_pub_->publish(color_msg);

  std_msgs::msg::Header depth_header;
  depth_header.frame_id = frame_id_;
  depth_header.stamp = now();
  sensor_msgs::msg::Image depth_msg;
  depth_msg.width = depth.get_width_pixels();
  depth_msg.height = depth.get_height_pixels();
  depth_msg.encoding = "mono16";
  depth_msg.is_bigendian = 1;
  depth_msg.step = sizeof(uint16_t)*depth.get_width_pixels();
  depth_msg.data.resize(depth.get_size());
  std::memcpy(depth_msg.data.data(), depth.get_buffer(), depth.get_size());
  depth_image_pub_->publish(depth_msg);
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<azure_kinect_ros2::AzureKinectNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
