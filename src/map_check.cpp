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

cv::Mat MapCheck::makeFieldCostMat(double size_x,double size_y, float cell_size){
  cv::Mat cost_mat((int)(size_y/cell_size), (int)(size_x/cell_size), CV_8S, cv::Scalar(-1));
  for (int y = 0; y < cost_mat.size().height; y++){
    for (int x = 0; x < cost_mat.size().width; x++){
      if(cost_mat.at<int8_t>(y,x)!=-1)std::cout<<"("<<x<<","<<y<<")"<<std::endl;
    }
  }
  return cost_mat;
}

Eigen::Vector2i MapCheck::m2pix(Eigen::Vector2d m_point,cv::Mat cost_mat, float cell_size){
  Eigen::Vector2i pix_point;
  // std::cout<<"oppai"<<std::endl;
  pix_point.x()=((m_point.x()/cell_size)+(cost_mat.size().width/2));
  pix_point.y()=((m_point.y()/cell_size)+(cost_mat.size().height/2));
  // std::cout<<pix_point.x()<<std::endl;
  // std::cout<<pix_point.y()<<std::endl;
  return pix_point;
}

// Eigen::Vector2i MapCheck::mm2pix(Eigen::Vector2d mm_point,cv::Mat cost_mat,float cell_size){
//   Eigen::Vector2i pix_point;
//   pix_point.x()=((mm_point.x()/1000)/cell_size)+(cost_mat.size().width/2);
//   pix_point.y()=((mm_point.y()/1000)/cell_size)+(cost_mat.size().height/2);
//   return pix_point;
// }

nav_msgs::msg::OccupancyGrid MapCheck::getMap(cv::Mat cost_mat,float cell_size){
  //map出力
  geometry_msgs::msg::Pose origin;
  origin.position.x=-cell_size*cost_mat.size().width/2;//[m]
  origin.position.y=-cell_size*cost_mat.size().height/2;//[m]
  origin.position.z=0;
  origin.orientation.x=0;
  origin.orientation.y=0;
  origin.orientation.z=0;
  origin.orientation.w=1;
  nav_msgs::msg::OccupancyGrid map;

  map.header.frame_id = "map";
  map.info.resolution = cell_size;//[m/cells]float32
  map.info.width = cost_mat.size().width;//[cells]uint32
  map.info.height = cost_mat.size().height;//[cells]uint32
  map.info.origin = origin;//geometry_msgs/Pose
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  map.info.map_load_time = ros_clock.now();
  map.data.resize(cost_mat.size().width*cost_mat.size().height);
  int dc=0;
  for (int y = 0; y < cost_mat.size().height; y++){
    for (int x = 0; x < cost_mat.size().width; x++){
      //map.data[dc]=100*cost_mat.at<unsigned char>(y,x)/255;//img
      map.data[dc]=cost_mat.at<int8_t>(y,x);//mat
      //if(cost_mat.at<int>(y,x)!=-1)std::cout<<"("<<x<<","<<y<<")"<<std::endl;
      //if(map.data[dc]!=-1)std::cout<<"("<<x<<","<<y<<")"<<std::endl;
      dc++;
    }
  }
  return map;
}

void MapCheck::TimerCallback()
{
  cv::Mat cost_mat;

  Eigen::Vector2d p1,p2,p3,p4;
  p1.x()=-4.5;p1.y()=1;
  p2.x()=-3.5;p2.y()=1;
  p3.x()=-4.5;p3.y()=-1;
  p4.x()=-3.5;p4.y()=-1;

  cost_mat=MapCheck::makeFieldCostMat(size_x,size_y, cell_size);
  
  Eigen::Vector2i pix_1=MapCheck::m2pix(p1,cost_mat, cell_size);
  cost_mat.at<int8_t>(pix_1.y(),pix_1.x())=99;
  Eigen::Vector2i pix_2=MapCheck::m2pix(p2,cost_mat, cell_size);
  cost_mat.at<int8_t>(pix_2.y(),pix_2.x())=99;
  Eigen::Vector2i pix_3=MapCheck::m2pix(p3,cost_mat, cell_size);
  cost_mat.at<int8_t>(pix_3.y(),pix_3.x())=99;
  Eigen::Vector2i pix_4=MapCheck::m2pix(p4,cost_mat, cell_size);
  cost_mat.at<int8_t>(pix_4.y(),pix_4.x())=99;
  
  cv::Mat_<int> points(4, 2);
  points(0,0)=pix_1.x();points(0,1)=pix_1.y();//x,y
  points(1,0)=pix_2.x();points(1,1)=pix_2.y();//x,y
  points(2,0)=pix_3.x();points(2,1)=pix_3.y();//x,y
  points(3,0)=pix_4.x();points(3,1)=pix_4.y();//x,y
  cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
  cv::rectangle(cost_mat, brect.tl(), brect.br(), cv::Scalar(99), -1, 8);
  //cv::Mat cost_mat = (cv::Mat_<int>(3,4) << 10,20,30,40,50,60,50,40,30,20,10,0);
  //cv::Mat img = cv::imread("/home/adachi/Lena_gray.jpg");
  
  //cv::Mat cost_mat;
  //cv::cvtColor(img, cost_mat,cv::COLOR_BGR2GRAY);
  //cv::imshow("opencv_logo", cost_mat);
  //std::cout<<cost_mat<<std::endl;
  this->pub_map->publish(MapCheck::getMap(cost_mat,cell_size));
    
}
