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

#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <bitset>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <deskew/deskew.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace Eigen;

using namespace builtin_interfaces::msg;
using namespace dua_common_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;
using namespace rcl_interfaces::msg;
using namespace sensor_msgs;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;

namespace scan_deskewer
{

class ScanDeskewer : public dua_node::NodeBase
{
public:
  /**
   * @brief Builds a new ScanDeskewer node.
   *
   * @param opts Node options.
   *
   * @throws RuntimeError
   */
  ScanDeskewer(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

  /**
   * @brief Finalizes node operation.
   */
  ~ScanDeskewer();

private:
  /* Node initialization routines. */
  void init_parameters() override;
  void init_cgroups() override;
  void init_subscribers() override;
  void init_publishers() override;

  /**
   * @brief Routine to initialize node structures.
   */
  void init_internals();

  /* Node parameters. */
  int64_t motion_buffer_size_ = 0;
  double motion_gravity_ = 0.0;
  bool motion_use_imu_ = false;
  bool motion_use_twist_ = false;
  bool motion_use_odometry_ = false;
  bool pointcloud_from_laserscan_ = false;
  std::string pointcloud_labels_x_ = "";
  std::string pointcloud_labels_y_ = "";
  std::string pointcloud_labels_z_ = "";
  std::string pointcloud_labels_timestamp_ = "";
  bool tf_disable_ = false;
  bool tf_ignore_stamp_ = false;
  int64_t tf_timeout_ms_ = 0;

  /* Node variables. */
  std::unique_ptr<deskew::Deskewer> deskewer_;
  std::mutex mutex_;
  std::string sensor_frame_id_;
  std::string imu_frame_id_;
  std::string twist_frame_id_;
  std::string odometry_frame_id_;
  bool refresh_isometry_;
  Isometry3d imu_isometry_;
  Isometry3d twist_isometry_;
  Isometry3d odometry_isometry_;
  bool imu_isometry_valid_;
  bool twist_isometry_valid_;
  bool odometry_isometry_valid_;

  /* Publishers. */
  rclcpp::Publisher<PointCloud2>::SharedPtr output_pointcloud_pub_;

  /* Publishers topics. */
  static const std::string output_pub_topic_;

  /* Publishers routines. */
  /**
   * @brief Publish deskewed ROS2 pointclouds.
   *
   * @param pointcloud PointCloud2 to publish.
   */
  void publish(const PointCloud2 & pointcloud);

  /* Subscriptions. */
  rclcpp::Subscription<Imu>::SharedPtr motion_imu_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr motion_twist_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr motion_odometry_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr input_laserscan_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr input_pointcloud_sub_;

  /* Subscriptions topics. */
  static const std::string motion_imu_sub_topic_;
  static const std::string motion_twist_sub_topic_;
  static const std::string motion_odometry_sub_topic_;
  static const std::string input_sub_topic_;

  /* Subscriptions callback groups. */
  rclcpp::CallbackGroup::SharedPtr motion_imu_sub_cgroup_;
  rclcpp::CallbackGroup::SharedPtr motion_twist_sub_cgroup_;
  rclcpp::CallbackGroup::SharedPtr motion_odometry_sub_cgroup_;
  rclcpp::CallbackGroup::SharedPtr input_sub_cgroup_;

  /* Subscriptions callbacks. */
  /**
   * @brief Callback to integrate imu data into the navigation state.
   *
   * @param msg Imu message to parse.
   */
  void motion_imu_clbk(const Imu::SharedPtr msg);

  /**
   * @brief Callback to integrate twist data into the navigation state.
   *
   * @param msg TwistWithCovarianceStamped message to parse.
   */
  void motion_twist_clbk(const TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Callback to integrate odometry data into the navigation state.
   *
   * @param msg Odometry message to parse.
   */
  void motion_odometry_clbk(const Odometry::SharedPtr msg);

  /**
   * @brief Callback to process ROS2 laserscans.
   *
   * @param msg LaserScan message to parse.
   */
  void input_laserscan_clbk(const LaserScan::SharedPtr msg);

  /**
   * @brief Callback to process ROS2 pointclouds.
   *
   * @param msg PointCloud2 message to parse.
   */
  void input_pointcloud_clbk(PointCloud2::UniquePtr msg);

  /* Utility routines. */

  /**
   * @brief Retrieve a transform as an Eigen::Isometry3d.
   *
   * @param source Source frame.
   * @param target Target frame.
   * @param time Request timestamp.
   * @param transform Transform result.
   *
   * @return True if the transformation is available, false otherwise.
   */
  bool get_isometry(
    const std::string & source, const std::string & target,
    const rclcpp::Time & time, Isometry3d & isometry);

  /**
   * @brief Deskew ROS2 pointclouds.
   *
   * @param pointcloud PointCloud2 to deskew.
   * @param count Number of points in the pointcloud.
   */
  void deskew(PointCloud2 & pointcloud, size_t count);

  /**
   * @brief Update the isometry transformation matrix from TF server.
   *
   * @param source Source frame.
   * @param target Target frame.
   * @param time Request timestamp.
   * @param transform Transform result.
   *
   * @return True if the transformation is available, false otherwise
   */
  bool refresh_isometry(
    const std::string & source, const std::string & target,
    const rclcpp::Time & time, Isometry3d & isometry);

  /**
   * @brief Convert ROS2 timestamp into seconds.
   *
   * @param timestamp Time to convert.
   */
  double timestamp_to_seconds(const Time &stamp);
};

} // namespace scan_deskewer
