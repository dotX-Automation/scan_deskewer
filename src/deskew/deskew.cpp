/**
 * Deskew library implementation.
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

#include <deskew/deskew.hpp>

namespace deskew
{

Pose Pose::transform(const Isometry3d & isometry) {
  Pose res;

  res.pos = isometry * pos;
  res.quat = isometry.linear() * quat;

  return res;
}

Twist Twist::transform(const Isometry3d & isometry) {
  Twist res;

  res.ang = isometry.linear() * ang;
  res.lin = isometry.linear() * lin + isometry.translation().cross(res.ang);

  return res;
}

Imu Imu::transform(const Isometry3d & isometry) {
  Imu res;

  res.gyro = isometry.linear() * gyro;
  res.accel = isometry.linear() * (accel + gyro.cross(gyro.cross(isometry.translation())));

  return res;
}

Deskewer::Deskewer(size_t buffer_size)
{
  size_ = buffer_size + 1ul;
  used_ = 0ul;
  curr_ = 0ul;

  accel_ = Vector3d::Zero();
  times_.reserve(size_);
  twists_.reserve(size_);
  for(size_t i = 0ul; i < size_; i++) {
    times_.push_back(0.0);
    twists_.push_back(Twist());
  }

  corr_avail_ = false;
  corr_time_ = 0.0;
  corr_twist_ = Twist();
}

void Deskewer::clear()
{
  used_ = 0ul;
  curr_ = 0ul;

  accel_ = Vector3d::Zero();
  for(size_t i = 0ul; i < size_; i++) {
    times_.at(i) = 0.0;
    twists_.at(i) = Twist();
  }

  corr_avail_ = false;
  corr_time_ = 0.0;
  corr_twist_ = Twist();
}

bool Deskewer::fuse(double time, const Twist & twist)
{
  if (time <= last()) {
    return false;
  }

  if (used_ > 0ul) {
    size_t idx = last_idx();
    double dt = time - times_[idx];
    accel_ = (twist.lin - twists_[idx].lin) / dt;
  }

  times_.at(curr_) = time;
  twists_.at(curr_) = twist;

  curr_++;
  if (curr_ >= size_) {
    curr_ = 0ul;
  }

  if (used_ < size_) {
    used_++;
  }

  if (corr_avail_ && time > corr_time_) {
    correct(corr_time_, corr_twist_);
  }

  return true;
}

bool Deskewer::fuse(double time, const Imu & imu)
{
  if (time <= last()) {
    return false;
  }

  times_.at(curr_) = time;
  if (used_ == 0ul) {
    accel_ = imu.accel;
    twists_.at(curr_).lin = Vector3d::Zero();
    twists_.at(curr_).ang = imu.gyro;
  } else {
    double dt = time - last();
    Vector3d lin = 0.5 * (imu.accel + accel_) * dt;
    accel_ = imu.accel;
    twists_.at(curr_).lin = lin;
    twists_.at(curr_).ang = imu.gyro;
  }

  curr_++;
  if (curr_ >= size_) {
    curr_ = 0ul;
  }

  if (used_ < size_) {
    used_++;
  }

  if (corr_avail_ && time > corr_time_) {
    correct(corr_time_, corr_twist_);
  }

  return true;
}

bool Deskewer::correct(double time, const Twist & twist)
{
  if (used_ == 0ul || time < first()) {
    return false;
  } else if (time > last()) {
    if (size_ == 1ul) {
      fuse(time, twist);
    } else {
      corr_avail_ = true;
      corr_time_ = time;
      corr_twist_ = twist;
    }
    return true;
  } else {
    Twist eval = evaluate(time);
    Twist correction = Twist();
    correction.lin = twist.lin - eval.lin;
    correction.ang = twist.ang - eval.ang;
    for (size_t i = 0ul; i < used_; i++) {
      twists_[i].lin = twists_[i].lin + correction.lin;
      twists_[i].ang = twists_[i].ang + correction.ang;
    }
    return true;
  }
}

Twist Deskewer::evaluate(double time) const
{
  Twist res;

  std::vector<size_t> idxs = find(time);

  if (idxs.size() == 1ul) {
    res = twists_[idxs[0]];
  } else if (idxs.size() == 2ul) {
    double p = (time - times_[idxs[0]]) / (times_[idxs[1]] - times_[idxs[0]]);
    res.lin = twists_[idxs[0]].lin + p * (twists_[idxs[1]].lin - twists_[idxs[0]].lin);
    res.ang = twists_[idxs[1]].ang + p * (twists_[idxs[1]].ang - twists_[idxs[0]].ang);
  }

  return res;
}

std::vector<Twist> Deskewer::evaluate(const std::vector<double> & times) const
{
  std::vector<Twist> res = std::vector<Twist>();
  res.reserve(times.size());

  for (double t : times) {
    res.push_back(evaluate(t));
  }

  return res;
}

std::vector<Pose> Deskewer::solve(const std::vector<double> & times, const Pose & init) const
{
  std::vector<Pose> res = std::vector<Pose>();
  res.reserve(times.size());

  std::vector<double> sorted = times;
  std::sort(sorted.begin(), sorted.end());

  res.push_back(init);
  Pose pose = init;
  Twist twist_prev = evaluate(sorted[0]);
  for (size_t i = 1ul; i < times.size(); i++) {
    double dt = sorted[i] - sorted[i-1];

    Twist twist_next = evaluate(sorted[i]);
    Twist twist;
    twist.lin = 0.5 * (twist_prev.lin + twist_next.lin);
    twist.ang = 0.5 * (twist_prev.ang + twist_next.ang);
    twist_prev = twist_next;

    double real = std::cos(0.5 * twist.ang.norm() * dt);
    Vector3d imag = std::sin(0.5 * twist.ang.norm() * dt) * twist.ang.normalized();
    Quaterniond quat_omega = Quaterniond(real, imag.x(), imag.y(), imag.z());
    Quaterniond quat_next = (pose.quat * quat_omega).normalized();
    Quaterniond quat_rot = pose.quat.slerp(0.5, quat_next).normalized();

    pose.pos = pose.pos + quat_rot * twist.lin * dt;
    pose.quat = quat_next;

    res.push_back(pose);
  }

  return res;
}

std::vector<Vector3d> Deskewer::deskew(
  const std::vector<double> & times,
  const std::vector<Vector3d> & points,
  const Pose & init) const
{
  std::vector<Vector3d> res = std::vector<Vector3d>();
  res.reserve(times.size());

  std::vector<Pose> sol = solve(times, init);
  for (size_t i = 1ul; i < times.size(); i++) {
    Isometry3d iso = Isometry3d::Identity();
    iso.translate(sol[i].pos);
    iso.linear() = sol[i].quat.toRotationMatrix();
    res.push_back(iso * points[i]);
  }

  return res;
}

void Deskewer::deskew(
  const std::vector<double> & times,
  std::vector<Vector3d> & points,
  const Pose & init) const
{
  std::vector<Pose> sol = solve(times, init);
  for (size_t i = 1ul; i < times.size(); i++) {
    Isometry3d iso = Isometry3d::Identity();
    iso.translate(sol[i].pos);
    iso.linear() = sol[i].quat.toRotationMatrix();
    points[i] = iso * points[i];
  }
}

size_t Deskewer::first_idx() const
{
  if (used_ < size_) {
    return 0ul;
  } else {
    return curr_;
  }
}

size_t Deskewer::last_idx() const
{
  if (used_ == 0ul) {
    return 0ul;
  } else if (used_ < size_) {
    return used_ - 1ul;
  } else {
    return ((curr_ == 0ul) ? size_ : curr_) - 1ul;
  }
}

std::vector<size_t> Deskewer::find(double time) const
{
  std::vector<size_t> idxs = std::vector<size_t>();

  if (used_ == 0ul) {
    return idxs;
  }

  size_t idx = 0ul;
  if (used_ == 1) {
    idxs.push_back(idx);
    return idxs;
  }

  idx = first_idx();
  if (time <= times_[idx]) {
    idxs.push_back(idx);
    return idxs;
  }

  idx = last_idx();
  if (time >= times_[idx]) {
    idxs.push_back(idx);
    return idxs;
  }

  if (std::binary_search(times_.begin(), times_.end(), time)) {
    return idxs;
  }

  std::vector<double>::const_iterator it;
  if (used_ < size_) {
    it = std::lower_bound(times_.begin(), times_.begin() + used_, time);
  } else if (curr_ == 0ul) {
    it = std::lower_bound(times_.begin(), times_.end(), time);
  } else if (time > times_[0]) {
    it = std::lower_bound(times_.begin(), times_.begin() + curr_ - 1ul, time);
  } else {
    it = std::lower_bound(times_.begin() + curr_, times_.end(), time);
  }
  idx = it - times_.begin();

  idxs.push_back(idx);
  idxs.push_back((idx+1)%size_);
  return idxs;
}

} // namespace deskew
