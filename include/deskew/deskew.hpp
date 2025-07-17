/**
 * Deskew library header.
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

#include <algorithm>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

namespace deskew
{

/**
 * Holds position and orientation data.
 */
struct Pose
{
  Pose()
  : pos(Vector3d::Zero()), quat(Quaterniond::Identity()) {}

  Pose(const Vector3d & pos, const Quaterniond & quat)
  : pos(pos), quat(quat) {}

  Vector3d pos;
  Quaterniond quat;

  Pose transform(const Isometry3d & isometry);
};

/**
 * Holds linear and angular velocity data.
 */
struct Twist
{
  Twist()
  : lin(Vector3d::Zero()), ang(Vector3d::Zero()) {}

  Twist(const Vector3d & lin, const Vector3d & ang)
  : lin(lin), ang(ang) {}

  Vector3d lin;
  Vector3d ang;

  Twist transform(const Isometry3d & isometry);
};

/**
 * Holds IMU data.
 */
struct Imu
{
  Imu()
  : accel(Vector3d::Zero()), gyro(Vector3d::Zero()) {}

  Imu(const Vector3d & accel, const Vector3d & gyro)
  : accel(accel), gyro(gyro) {}

  Vector3d accel;
  Vector3d gyro;

  Imu transform(const Isometry3d & isometry);
};


/**
 * Deskewer class for handling of deskewing operations.
 */
class Deskewer
{
public:
  Deskewer(size_t buffer_size = 0ul);
  ~Deskewer() {}

  void clear();

  bool fuse(double time, const Twist & twist);
  bool fuse(double time, const Imu & imu);

  bool correct(double time, const Twist & twist);

  inline size_t size() const {return size_;}
  inline size_t used() const {return used_;}
  inline size_t curr() const {return curr_;}

  inline bool full() const {return used_ == size_;}
  inline bool empty() const {return used_ == 0;}

  inline double first() const {return times_.at(first_idx());}
  inline double last() const {return times_.at(last_idx());}

  Twist evaluate(double time) const;
  std::vector<Twist> evaluate(const std::vector<double> & times) const;

  std::vector<Pose> solve(const std::vector<double> & times, const Pose & init = Pose()) const;

  std::vector<Vector3d> deskew(
    const std::vector<double> & times,
    const std::vector<Vector3d> & points,
    const Pose & init = Pose()) const;

  void deskew(
    const std::vector<double> & times,
    std::vector<Vector3d> & points,
    const Pose & init = Pose()) const;

private:
  size_t size_;
  size_t used_;
  size_t curr_;

  Vector3d accel_;
  std::vector<double> times_;
  std::vector<Twist> twists_;

  bool corr_avail_;
  double corr_time_;
  Twist corr_twist_;

  size_t first_idx() const;
  size_t last_idx() const;
  std::vector<size_t> find(double time) const;
};

} // namespace deskew
