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

nav_msgs::msg::OccupancyGrid MapCheck::getMap(cv::Mat cost_mat){
  //map出力
  float res=0.1;
  geometry_msgs::msg::Pose origin;
  origin.position.x=res*cost_mat.size().width/2;//[m]
  origin.position.y=res*cost_mat.size().height/2;//[m]
  origin.position.z=0;
  origin.orientation.x=0;
  origin.orientation.y=0;
  origin.orientation.z=1;
  origin.orientation.w=0;
  nav_msgs::msg::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.resolution = res;//[m/cells]float32
  map.info.width = cost_mat.size().width;//[cells]uint32
  map.info.height = cost_mat.size().height;//[cells]uint32
  map.info.origin = origin;//geometry_msgs/Pose
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  map.info.map_load_time = ros_clock.now();
  map.data.resize(cost_mat.size().width*cost_mat.size().height);
  int dc=0;
  for (int y = 0; y < cost_mat.size().height; y++){
    for (int x = 0; x < cost_mat.size().width; x++){
      map.data[dc]=100*cost_mat.at<unsigned char>(y,x)/255;//img
      //map.data[dc]=cost_mat.at<int>(y,x);//mat
      dc++;
    }
  }
  return map;
}


void MapCheck::TimerCallback()
{
    //cv::Mat cost_mat = (cv::Mat_<int>(3,4) << 10,20,30,40,50,60,50,40,30,20,10,0);
    cv::Mat img = cv::imread("/home/adachi/Lena_gray.jpg");
    if (img.empty() == false) {
      cv::Mat cost_mat;
      cv::cvtColor(img, cost_mat,cv::COLOR_BGR2GRAY);
      //cvShowImage("opencv_logo", cost_mat);
      this->pub_map->publish(MapCheck::getMap(cost_mat));
    }
}
