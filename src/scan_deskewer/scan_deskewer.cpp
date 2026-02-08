/**
 * Scan Deskewer node definition.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * July 4, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <scan_deskewer/scan_deskewer.hpp>

namespace scan_deskewer
{

ScanDeskewer::ScanDeskewer(const rclcpp::NodeOptions & opts)
: NodeBase("scan_deskewer", opts, true)
{
  dua_init_node();

  init_internals();

  RCLCPP_INFO(get_logger(), "Node initialized");
}

ScanDeskewer::~ScanDeskewer()
{}

void ScanDeskewer::init_cgroups()
{
  if (motion_use_imu_) {
    motion_imu_sub_cgroup_ = dua_create_exclusive_cgroup();
  }

  if (motion_use_twist_) {
    motion_twist_sub_cgroup_ = dua_create_exclusive_cgroup();
  }

  if (motion_use_odometry_) {
    motion_odometry_sub_cgroup_ = dua_create_exclusive_cgroup();
  }

  input_sub_cgroup_ = dua_create_exclusive_cgroup();
}


void ScanDeskewer::init_subscribers()
{
  if (motion_use_imu_) {
    motion_imu_sub_ = dua_create_subscription<Imu>(
      motion_imu_sub_topic_,
      std::bind(
        &ScanDeskewer::motion_imu_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      motion_imu_sub_cgroup_);
  }

  if (motion_use_twist_) {
    motion_twist_sub_ = dua_create_subscription<TwistWithCovarianceStamped>(
      motion_twist_sub_topic_,
      std::bind(
        &ScanDeskewer::motion_twist_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      motion_twist_sub_cgroup_);
  }

  if (motion_use_odometry_) {
    motion_odometry_sub_ = dua_create_subscription<Odometry>(
      motion_odometry_sub_topic_,
      std::bind(
        &ScanDeskewer::motion_odometry_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      motion_odometry_sub_cgroup_);
  }

  if (pointcloud_from_laserscan_) {
    input_laserscan_sub_ = dua_create_subscription<LaserScan>(
      input_sub_topic_,
      std::bind(
        &ScanDeskewer::input_laserscan_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      input_sub_cgroup_);
  } else {
    input_pointcloud_sub_ = dua_create_subscription<PointCloud2>(
      input_sub_topic_,
      std::bind(
        &ScanDeskewer::input_pointcloud_clbk,
        this,
        std::placeholders::_1),
      dua_qos::Reliable::get_datum_qos(),
      input_sub_cgroup_);
  }
}

void ScanDeskewer::init_publishers()
{
  output_pointcloud_pub_ = dua_create_publisher<PointCloud2>(
    output_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
}

void ScanDeskewer::init_internals()
{
  deskewer_ = std::make_unique<deskew::Deskewer>(motion_buffer_size_);
  sensor_frame_id_ = "";
  imu_frame_id_ = "";
  twist_frame_id_ = "";
  odometry_frame_id_ = "";
  imu_isometry_ = Isometry3d::Identity();
  twist_isometry_ = Isometry3d::Identity();
  odometry_isometry_ = Isometry3d::Identity();
  imu_isometry_valid_ = false;
  twist_isometry_valid_ = false;
  odometry_isometry_valid_ = false;
}

} // namespace scan_deskewer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(scan_deskewer::ScanDeskewer)
