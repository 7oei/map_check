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

#include "map_check/map_check.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

MapCheck::MapCheck()
: Node("map_check")
{
  pub_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

  // Timer
  control_period=0.1;
  auto timer_duration = std::chrono::duration<double>(control_period);
  timer = create_wall_timer(timer_duration, std::bind(&MapCheck::TimerCallback, this));
}

geometry_msgs::msg::Pose2D MapCheck::Pose2Pose2D(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Pose2D pose2d;
  pose2d.x = pose.position.x;
  pose2d.y = pose.position.y;
  pose2d.theta = tf2::getYaw(pose.orientation);
  return pose2d;
}

geometry_msgs::msg::Pose MapCheck::Pose2D2Pose(const geometry_msgs::msg::Pose2D & pose2d)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = pose2d.x;
  pose.position.y = pose2d.y;
  pose.position.z = 0;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, pose2d.theta);
  pose.orientation = tf2::toMsg(quat);
  return pose;
}

double MapCheck::normalizeAnglePositive(double angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double MapCheck::normalizeAngle(double angle)
{
  double a = MapCheck::normalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}


void MapCheck::TimerCallback()
{
    //map出力
    geometry_msgs::msg::Pose origin;
    origin.position.x=-5;//[m]
    origin.position.y=-5;//[m]
    origin.position.z=0;
    origin.orientation.x=0;
    origin.orientation.y=0;
    origin.orientation.z=0;
    origin.orientation.w=1;
    nav_msgs::msg::OccupancyGrid map;

    map.header.frame_id = "map";
    map.info.resolution = 0.1;//[m/cells]float32
    map.info.width = 100;//[cells]uint32
    map.info.height = 100;//[cells]uint32
    map.info.origin = origin;//geometry_msgs/Pose
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    map.info.map_load_time = ros_clock.now();
    for(int i=1;i<=10000;i++){
    map.data.push_back(1+i/103);//int8(0~100)
    }

    this->pub_map->publish(map);
}
