/**
 * Scan Deskewer subscriptions ruotine.
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

void ScanDeskewer::motion_imu_clbk(const Imu::SharedPtr msg)
{
  if (!tf_disable_) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Imu frame_id is empty");
      return;
    }

    if (msg->header.frame_id != imu_frame_id_) {
      imu_frame_id_ = msg->header.frame_id;
      imu_isometry_valid_ = refresh_isometry(
        sensor_frame_id_,
        imu_frame_id_,
        msg->header.stamp,
        imu_isometry_);
    }
  }

  double time = timestamp_to_seconds(msg->header.stamp);
  deskew::Imu imu;
  imu.accel.x() = msg->linear_acceleration.x;
  imu.accel.y() = msg->linear_acceleration.y;
  imu.accel.z() = msg->linear_acceleration.z;
  imu.gyro.x() = msg->angular_velocity.x;
  imu.gyro.y() = msg->angular_velocity.y;
  imu.gyro.z() = msg->angular_velocity.z;

  if (motion_gravity_ > 0.0) {
    Quaterniond quat = Quaterniond(
    msg->orientation.w,
    msg->orientation.x,
    msg->orientation.y,
    msg->orientation.z);

    imu.accel += quat * Vector3d(0.0, 0.0, -motion_gravity_);
  }

  if (!tf_disable_) {
    if (imu_isometry_valid_) {
      imu = imu.transform(imu_isometry_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Imu isometry not valid");
      return;
    }
  }

  {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_->fuse(time, imu);
  }
}

void ScanDeskewer::motion_twist_clbk(const TwistWithCovarianceStamped::SharedPtr msg)
{
  if (!tf_disable_) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Twist frame_id is empty");
      return;
    }

    if (msg->header.frame_id != twist_frame_id_) {
      twist_frame_id_ = msg->header.frame_id;
      twist_isometry_valid_ = refresh_isometry(
        sensor_frame_id_,
        twist_frame_id_,
        msg->header.stamp,
        twist_isometry_);
    }
  }

  double time = timestamp_to_seconds(msg->header.stamp);
  deskew::Twist twist;
  twist.lin.x() = msg->twist.twist.linear.x;
  twist.lin.y() = msg->twist.twist.linear.y;
  twist.lin.z() = msg->twist.twist.linear.z;
  twist.ang.x() = msg->twist.twist.angular.x;
  twist.ang.y() = msg->twist.twist.angular.y;
  twist.ang.z() = msg->twist.twist.angular.z;

  if (!tf_disable_) {
    if (twist_isometry_valid_) {
      twist = twist.transform(twist_isometry_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Twist isometry not valid");
      return;
    }
  }

  if (motion_use_imu_ && !motion_use_odometry_) {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_->correct(time, twist);
  } else {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_->fuse(time, twist);
  }
}

void ScanDeskewer::motion_odometry_clbk(const Odometry::SharedPtr msg)
{
  if (!tf_disable_) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry frame_id is empty");
      return;
    }

    if (msg->header.frame_id != odometry_frame_id_) {
      odometry_frame_id_ = msg->header.frame_id;
      odometry_isometry_valid_ = refresh_isometry(
        sensor_frame_id_,
        odometry_frame_id_,
        msg->header.stamp,
        odometry_isometry_);
    }
  }

  double time = timestamp_to_seconds(msg->header.stamp);
  deskew::Twist twist;
  twist.lin.x() = msg->twist.twist.linear.x;
  twist.lin.y() = msg->twist.twist.linear.y;
  twist.lin.z() = msg->twist.twist.linear.z;
  twist.ang.x() = msg->twist.twist.angular.x;
  twist.ang.y() = msg->twist.twist.angular.y;
  twist.ang.z() = msg->twist.twist.angular.z;

  if (!tf_disable_) {
    if (odometry_isometry_valid_) {
      twist = twist.transform(odometry_isometry_);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry isometry not valid");
      return;
    }
  }

  if (motion_use_imu_ || motion_use_twist_) {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_->correct(time, twist);
  } else {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_->fuse(time, twist);
  }
}


void ScanDeskewer::input_laserscan_clbk(const LaserScan::SharedPtr msg)
{
  if (!tf_disable_) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "LaserScan frame_id is empty");
      return;
    }

    if (msg->header.frame_id != sensor_frame_id_) {
      sensor_frame_id_ = msg->header.frame_id;

      if (motion_use_imu_) {
        imu_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          imu_frame_id_,
          msg->header.stamp,
          imu_isometry_);
      }
      if (motion_use_twist_) {
        twist_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          twist_frame_id_,
          msg->header.stamp,
          twist_isometry_);
      }
      if (motion_use_odometry_) {
        odometry_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          odometry_frame_id_,
          msg->header.stamp,
          odometry_isometry_);
      }
    }
  }

  PointCloud2 pointcloud;
  pointcloud.header = msg->header;

  size_t count = msg->ranges.size();

  PointCloud2Modifier modifier(pointcloud);
  modifier.setPointCloud2Fields(5,
    pointcloud_labels_x_.c_str(), 1, PointField::FLOAT32,
    pointcloud_labels_y_.c_str(), 1, PointField::FLOAT32,
    pointcloud_labels_z_.c_str(), 1, PointField::FLOAT32,
    "intensity", 1, PointField::FLOAT32,
    pointcloud_labels_timestamp_.c_str(), 1, PointField::FLOAT32);
  modifier.reserve(count);

  PointCloud2Iterator<float> x_iter(pointcloud, pointcloud_labels_x_);
  PointCloud2Iterator<float> y_iter(pointcloud, pointcloud_labels_y_);
  PointCloud2Iterator<float> z_iter(pointcloud, pointcloud_labels_z_);
  PointCloud2Iterator<float> intensity_iter(pointcloud, "intensity");
  PointCloud2Iterator<double> timestamp_iter(pointcloud, pointcloud_labels_timestamp_);

  double x, y, z = 0.0;
  double angle = msg->angle_min;
  double timestamp = timestamp_to_seconds(msg->header.stamp);
  for (size_t i = 0; i < count; ++i) {
    x = msg->ranges[i] * std::cos(angle);
    y = msg->ranges[i] * std::sin(angle);

    *x_iter = (float) x;
    *y_iter = (float) y;
    *z_iter = (float) z;
    *intensity_iter = msg->intensities[i];
    *timestamp_iter = timestamp;

    angle = std::fmin(angle + msg->angle_increment, msg->angle_max);
    timestamp += msg->time_increment;

    ++x_iter;
    ++y_iter;
    ++z_iter;
    ++intensity_iter;
    ++timestamp_iter;
  }

  deskew(pointcloud, count);
  publish(pointcloud);
}


void ScanDeskewer::input_pointcloud_clbk(PointCloud2::UniquePtr msg)
{
  if (!tf_disable_) {
    if (msg->header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "PointCloud2 frame_id is empty");
      return;
    }

    if (msg->header.frame_id != sensor_frame_id_) {
      sensor_frame_id_ = msg->header.frame_id;

      if (motion_use_imu_) {
        imu_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          imu_frame_id_,
          msg->header.stamp,
          imu_isometry_);
      }
      if (motion_use_twist_) {
        twist_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          twist_frame_id_,
          msg->header.stamp,
          twist_isometry_);
      }
      if (motion_use_odometry_) {
        odometry_isometry_valid_ = refresh_isometry(
          sensor_frame_id_,
          odometry_frame_id_,
          msg->header.stamp,
          odometry_isometry_);
      }
    }
  }

  bool t_found = false;
  bool x_found = false;
  bool y_found = false;
  bool z_found = false;
  size_t found = 0ul;
  size_t count = 0ul;

  for (size_t i = 0ul; i < msg->fields.size(); i++) {
    bool field_found = false;
    if (msg->fields[i].name == pointcloud_labels_timestamp_) {
      t_found = true;
      field_found = true;
    } else if (msg->fields[i].name == pointcloud_labels_x_) {
      x_found = true;
      field_found = true;
    } else if (msg->fields[i].name == pointcloud_labels_y_) {
      y_found = true;
      field_found = true;
    } else if (msg->fields[i].name == pointcloud_labels_z_) {
      z_found = true;
      field_found = true;
    }

    if (field_found) {
      if (found == 0ul) {
        count = msg->fields[i].count;
      } else if (count != msg->fields[i].count) {
        count = 0ul;
        break;
      }
      found++;
    }
  }

  bool count_check = count == 1ul;
  bool found_check = t_found && x_found && y_found && z_found && found == 4ul;

  if (!count_check || !found_check) {
    std::string error = !count_check ? "fields elements count is zero or inconsistent" : "not all fields found";
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "PointCloud2 not valid: %s", error.c_str());
    return;
  }

  deskew(*msg, msg->height * msg->width);
  publish(*msg);
}

} // namespace scan_deskewer
