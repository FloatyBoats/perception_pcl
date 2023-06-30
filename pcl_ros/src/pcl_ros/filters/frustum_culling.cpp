/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: FrustumCulling.cpp
 *
 */

#include "pcl_ros/filters/frustum_culling.hpp"

pcl_ros::FrustumCulling::FrustumCulling(const rclcpp::NodeOptions & options)
: Filter("FrustumCullingNode", options)
{
  // This both declares and initializes the input and output frames
  use_frame_params();

  rcl_interfaces::msg::ParameterDescriptor vertical_fov_desc;
  vertical_fov_desc.name = "vertical_fov";
  vertical_fov_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  vertical_fov_desc.description =
    "Vertical FOV of camera in degrees.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 180.0;
    vertical_fov_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(vertical_fov_desc.name, rclcpp::ParameterValue(90.0), vertical_fov_desc);

  rcl_interfaces::msg::ParameterDescriptor horizontal_fov_desc;
  horizontal_fov_desc.name = "horizontal_fov";
  horizontal_fov_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  horizontal_fov_desc.description =
    "Horizontal FOV of camera in degrees.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 180.0;
    horizontal_fov_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(horizontal_fov_desc.name, rclcpp::ParameterValue(90.0), horizontal_fov_desc);

  rcl_interfaces::msg::ParameterDescriptor near_plane_dist_desc;
  near_plane_dist_desc.name = "near_plane_distance";
  near_plane_dist_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  near_plane_dist_desc.description =
    "Near plane distance.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1000.0;
    near_plane_dist_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(near_plane_dist_desc.name, rclcpp::ParameterValue(0.0), near_plane_dist_desc);

  rcl_interfaces::msg::ParameterDescriptor far_plane_dist_desc;
  far_plane_dist_desc.name = "far_plane_distance";
  far_plane_dist_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  far_plane_dist_desc.description =
    "Near plane distance.";
  {
    rcl_interfaces::msg::FloatingPointRange float_range;
    float_range.from_value = 0.0;
    float_range.to_value = 1000.0;
    far_plane_dist_desc.floating_point_range.push_back(float_range);
  }
  declare_parameter(far_plane_dist_desc.name, rclcpp::ParameterValue(1000.0), far_plane_dist_desc);

  rcl_interfaces::msg::ParameterDescriptor keep_organized_desc;
  keep_organized_desc.name = "keep_organized";
  keep_organized_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  keep_organized_desc.description =
    "Set whether the filtered points should be kept and set to NaN, "
    "or removed from the PointCloud, thus potentially breaking its organized structure.";
  declare_parameter(keep_organized_desc.name, rclcpp::ParameterValue(false), keep_organized_desc);

  rcl_interfaces::msg::ParameterDescriptor negative_desc;
  negative_desc.name = "negative";
  negative_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  negative_desc.description =
    "Set whether the inliers should be returned (true) or the outliers (false).";
  declare_parameter(negative_desc.name, rclcpp::ParameterValue(false), negative_desc);

  const std::vector<std::string> param_names {
    vertical_fov_desc.name,
    keep_organized_desc.name,
    negative_desc.name,
  };

  callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &FrustumCulling::config_callback, this,
      std::placeholders::_1));

  config_callback(get_parameters(param_names));

  // TODO(daisukes): lazy subscription after rclcpp#2060
  subscribe();
}

void
pcl_ros::FrustumCulling::filter(
  const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto pcl2_input = std::make_shared<pcl::PCLPointCloud2>();
  pcl_conversions::toPCL(*(input), *(pcl2_input));

  auto pcl_input = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
initUndistortRectifyMap
  pcl::PointCloud<pcl::PointXYZRGB> pcl_output;
  impl_.filter(pcl_output);

  pcl::PCLPointCloud2 pcl2_output;
  pcl::toPCLPointCloud2(pcl_output, pcl2_output);
  pcl_conversions::moveFromPCL(pcl2_output, output);
}

//////////////////////////////////////////////////////////////////////////////////////////////

rcl_interfaces::msg::SetParametersResult
pcl_ros::FrustumCulling::config_callback(const std::vector<rclcpp::Parameter> & params)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // TODO: how should this pose be set?
  Eigen::Matrix4f cam_pose;
  cam_pose << -0.4480736, -0.8939967,  0.0000000, 0,
               0.8939967, -0.4480736,  0.0000000, 0,
               0.0000000,  0.0000000,  1.0000000, 0,
               0,          0,          0,         1;
  impl_.setCameraPose(cam_pose);

  for (const rclcpp::Parameter & param : params) {
    if (param.get_name() == "vertical_fov") {
      impl_.setVerticalFOV(param.as_double());
    }
    if (param.get_name() == "horizontal_fov") {
      impl_.setHorizontalFOV(param.as_double());
    }
    if (param.get_name() == "near_plane_distance") {
      impl_.setNearPlaneDistance(param.as_double());
    }
    if (param.get_name() == "far_plane_distance") {
      impl_.setFarPlaneDistance(param.as_double());
    }
    if (param.get_name() == "negative") {
      // Check the current value for the negative flag
      if (impl_.getNegative() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter negative flag to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setNegative(param.as_bool());
      }
    }
    if (param.get_name() == "keep_organized") {
      // Check the current value for keep_organized
      if (impl_.getKeepOrganized() != param.as_bool()) {
        RCLCPP_DEBUG(
          get_logger(), "Setting the filter keep_organized value to: %s.",
          param.as_bool() ? "true" : "false");
        // Call the virtual method in the child
        impl_.setKeepOrganized(param.as_bool());
      }
    }
  }

  // Range constraints are enforced by rclcpp::Parameter.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros::FrustumCulling)
