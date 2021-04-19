// Copyright (c) 2019-2020 Scramble
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef MAP_CHECK__MAP_CHECK_HPP_
#define MAP_CHECK__MAP_CHECK_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/utilities.hpp"

class MapCheck : public rclcpp::Node
{
public:
  MapCheck();
  // コールバック
  void TimerCallback();
  // 基本幾何計算
  double normalizeAnglePositive(double angle);
  double normalizeAngle(double angle);
  nav_msgs::msg::OccupancyGrid getMap(cv::Mat cost_mat);
  geometry_msgs::msg::Pose2D Pose2Pose2D(const geometry_msgs::msg::Pose & pose);
  geometry_msgs::msg::Pose Pose2D2Pose(const geometry_msgs::msg::Pose2D & pose2d);

private:
  double control_period;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map;
  rclcpp::TimerBase::SharedPtr timer;
};

#endif  // MAP_CHECK__MAP_CHECK_HPP_
