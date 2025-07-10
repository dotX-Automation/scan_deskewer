/**
 * Scan Deskewer utilities.
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


void ScanDeskewer::deskew(PointCloud2 & pointcloud, size_t count)
{
  std::vector<double> times;
  std::vector<Vector3d> points;

  times.reserve(count);
  points.reserve(count);

  PointCloud2Iterator<double> timestamp_iter(pointcloud, pointcloud_labels_timestamp_);
  PointCloud2Iterator<float> x_iter(pointcloud, pointcloud_labels_x_);
  PointCloud2Iterator<float> y_iter(pointcloud, pointcloud_labels_y_);
  PointCloud2Iterator<float> z_iter(pointcloud, pointcloud_labels_z_);
  for (size_t i = 0; i < count; ++i) {
    times.push_back(*timestamp_iter);
    points.push_back(Vector3d(*x_iter, *y_iter, *z_iter));

    ++timestamp_iter;
    ++x_iter;
    ++y_iter;
    ++z_iter;
  }

  {
    std::lock_guard<std::mutex> guard(mutex_);
    deskewer_.deskew(times, points);
  }

  PointCloud2Iterator<float> x_iter_deskew(pointcloud, pointcloud_labels_x_);
  PointCloud2Iterator<float> y_iter_deskew(pointcloud, pointcloud_labels_y_);
  PointCloud2Iterator<float> z_iter_deskew(pointcloud, pointcloud_labels_z_);
  for (size_t i = 0; i < count; ++i) {
    *x_iter_deskew = points[i].x();
    *y_iter_deskew = points[i].y();
    *z_iter_deskew = points[i].z();

    ++x_iter_deskew;
    ++y_iter_deskew;
    ++z_iter_deskew;
  }
}


bool ScanDeskewer::refresh_isometry(
  const std::string & source, const std::string & target,
  const rclcpp::Time & time, Isometry3d & isometry)
{
  if(source.empty()) {
    return false;
  }
  
  if(target.empty()) {
    return false;
  }

  return get_transform(source, target, time, isometry);
}


double ScanDeskewer::timestamp_to_seconds(const Time &stamp)
{
  return ((double) stamp.sec) + 1e-9 * ((double) stamp.nanosec);
}


} // namespace scan_deskewer
