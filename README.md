# scan_deskewer

Node to deskew laserscans and pointclouds.

## Usage

The ``scan_deskewer`` node allows to mitigate the skew phenomena in ``LaserScan`` or ``PointCloud2`` messages.
The node uses at least one of the following messages to compute agent motion and deskew the points in the scan.

- **``Imu``**: reading from an IMU mounted on the agent. The orientation fields must be filled in to correctly remove gravity from linear acceleration.
- **``TwistWithCovarianceStamped``**: reading from an onboard odometric sensor (like encoders or optical flow). Both the linear and angular velocities must be expressed with respect to the body frame.
- **``Odometry``**: navigation solution (like the results from the EKF). Both the linear and angular velocities must be expressed with respect to the body frame.

The following table summarizes if a sensor is used to integrate the position or to correct the velocity drift, for all the possible combinations.

| Use IMU | Use Twist | Use Odometry | Used for integration | Used for corrections | Comments
|-|-|-|-|-|-
| x | x | x | IMU, Twist | Odometry | Redundant
| x | x |   | IMU        | Twist    | OK
|   | x | x | Twist      | Odometry | Fine if Twist frequency is greater than Odometry frequency
| x |   | x | IMU        | Odometry | OK
| x |   |   | IMU        | //       | Traditional method, not recommended due to drifting
|   | x |   | Twist      | //       | OK
|   |   | x | Odometry   | //       | OK

The parameter ``buffer_size`` determines how many measurements (cumulative) from the sensors used for integration are used to compute the agent motion. If set to ``0``, the integration is disabled and only the last available velocity is used to compute the agent motion.

## Copyright and License

Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.
