/**
 * Scan Deskewer node constants.
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

/* Publishers topics. */
const std::string ScanDeskewer::output_pub_topic_ = "~/output";

/* Subscriptions topics. */
const std::string ScanDeskewer::motion_imu_sub_topic_ = "/imu";
const std::string ScanDeskewer::motion_twist_sub_topic_ = "/twist";
const std::string ScanDeskewer::motion_odometry_sub_topic_ = "/odometry";
const std::string ScanDeskewer::input_sub_topic_ = "/input";

} // namespace scan_deskewer
