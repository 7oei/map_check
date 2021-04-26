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

cv::Mat MapCheck::makeFieldCostMat(double size_x,double size_y,int value, float cell_size){
  cv::Mat cost_mat((int)(size_y/cell_size), (int)(size_x/cell_size), CV_8S, cv::Scalar(value));
  return cost_mat;
}

Eigen::Vector2i MapCheck::m2pix(Eigen::Vector2d m_point,cv::Mat cost_mat, float cell_size){
  Eigen::Vector2i pix_point;
  pix_point.x()=((m_point.x()/cell_size)+(cost_mat.size().width/2));
  pix_point.y()=((m_point.y()/cell_size)+(cost_mat.size().height/2));
  return pix_point;
}

// Eigen::Vector2i MapCheck::mm2pix(Eigen::Vector2d mm_point,cv::Mat cost_mat,float cell_size){
//   Eigen::Vector2i pix_point;
//   pix_point.x()=((mm_point.x()/1000)/cell_size)+(cost_mat.size().width/2);
//   pix_point.y()=((mm_point.y()/1000)/cell_size)+(cost_mat.size().height/2);
//   return pix_point;
// }

void MapCheck::setInsideRectangle(cv::Mat *cost_mat,int value,Eigen::Vector2d p1,Eigen::Vector2d p2,Eigen::Vector2d p3,Eigen::Vector2d p4){
  Eigen::Vector2i pix_1=MapCheck::m2pix(p1,*cost_mat, cell_size);
  Eigen::Vector2i pix_2=MapCheck::m2pix(p2,*cost_mat, cell_size);
  Eigen::Vector2i pix_3=MapCheck::m2pix(p3,*cost_mat, cell_size);
  Eigen::Vector2i pix_4=MapCheck::m2pix(p4,*cost_mat, cell_size);
  cv::Mat_<int> points(4, 2);
  points(0,0)=pix_1.x();points(0,1)=pix_1.y();//x,y
  points(1,0)=pix_2.x();points(1,1)=pix_2.y();//x,y
  points(2,0)=pix_3.x();points(2,1)=pix_3.y();//x,y
  points(3,0)=pix_4.x();points(3,1)=pix_4.y();//x,y
  cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
  cv::rectangle(*cost_mat, brect.tl(), brect.br(), cv::Scalar(value), -1, 8);
}

void MapCheck::setOutsideRectangle(cv::Mat *cost_mat,int value,Eigen::Vector2d p1,Eigen::Vector2d p2,Eigen::Vector2d p3,Eigen::Vector2d p4){
  cv::Mat mask(cost_mat->size().height, cost_mat->size().width, CV_8S, cv::Scalar(value));
  Eigen::Vector2i pix_1=MapCheck::m2pix(p1,mask, cell_size);
  Eigen::Vector2i pix_2=MapCheck::m2pix(p2,mask, cell_size);
  Eigen::Vector2i pix_3=MapCheck::m2pix(p3,mask, cell_size);
  Eigen::Vector2i pix_4=MapCheck::m2pix(p4,mask, cell_size);
  cv::Mat_<int> points(4, 2);
  points(0,0)=pix_1.x();points(0,1)=pix_1.y();//x,y
  points(1,0)=pix_2.x();points(1,1)=pix_2.y();//x,y
  points(2,0)=pix_3.x();points(2,1)=pix_3.y();//x,y
  points(3,0)=pix_4.x();points(3,1)=pix_4.y();//x,y
  cv::Rect brect = cv::boundingRect(cv::Mat(points).reshape(2));
  cv::rectangle(mask, brect.tl(), brect.br(), cv::Scalar(0), -1, 8);
  bitwise_or(mask, *cost_mat, *cost_mat);
}

void MapCheck::setInsideCircle(cv::Mat *cost_mat,int value,Eigen::Vector2d p,float r){
  Eigen::Vector2i pix=MapCheck::m2pix(p,*cost_mat, cell_size);
  cv::circle(*cost_mat, cv::Point(pix.x(), pix.y()), r/cell_size, cv::Scalar(value), -1, 8);
}

void MapCheck::setInsideLine(cv::Mat *cost_mat,int value,Eigen::Vector2d p1,Eigen::Vector2d p2,float r){
  Eigen::Vector2i pix_1=MapCheck::m2pix(p1,*cost_mat, cell_size);
  Eigen::Vector2i pix_2=MapCheck::m2pix(p2,*cost_mat, cell_size);
  cv::line(*cost_mat, cv::Point(pix_1.x(), pix_1.y()), cv::Point(pix_2.x(), pix_2.y()), cv::Scalar(value), 2*r/cell_size, 8);
}

void MapCheck::setInsideOsiri(cv::Mat *cost_mat,int value,Eigen::Vector2d p,float angle,float r){
  Eigen::Vector2i pix=MapCheck::m2pix(p,*cost_mat, cell_size);
  // # ellipse(画像, 中心座標, Size(x径, y径), 楕円の回転角度, 始点角度, 終点角度, 色, 線幅, 連結)
  cv::ellipse(*cost_mat, cv::Point(pix.x(), pix.y()), cv::Size(2*r/cell_size, 2*r/cell_size), 0, angle-90, angle+90, cv::Scalar(value), -1, 8);
  Eigen::Vector2i v1;
  v1.x()=2*0.55*r/cell_size*cos(MapCheck::normalizeAngle((angle-90)*M_PI/180));
  v1.y()=2*0.55*r/cell_size*sin(MapCheck::normalizeAngle((angle-90)*M_PI/180));
  cv::ellipse(*cost_mat, cv::Point(pix.x()+v1.x(), pix.y()+v1.y()), cv::Size(0.45*2*r/cell_size, 0.45*2*r/cell_size), 180, angle-90, angle+90, cv::Scalar(value), -1, 8);
  cv::ellipse(*cost_mat, cv::Point(pix.x()-v1.x(), pix.y()-v1.y()), cv::Size(0.45*2*r/cell_size, 0.45*2*r/cell_size), 180, angle-90, angle+90, cv::Scalar(value), -1, 8);
  cv::ellipse(*cost_mat, cv::Point(pix.x(), pix.y()), cv::Size(0.1*2*r/cell_size, 0.1*2*r/cell_size), 0, angle-90, angle+90, cv::Scalar(0), -1, 8);
}

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

  Eigen::Vector2d p1,p2,p3,p4;//ディフェンスエリア
  p1.x()=-4.5;p1.y()=1;
  p2.x()=-3.5;p2.y()=1;
  p3.x()=-4.5;p3.y()=-1;
  p4.x()=-3.5;p4.y()=-1;

  Eigen::Vector2d p5,p6,p7,p8;//フィールドライン
  p5.x()=-4.5;p5.y()=3;
  p6.x()=4.5;p6.y()=3;
  p7.x()=-4.5;p7.y()=-3;
  p8.x()=4.5;p8.y()=-3;

  Eigen::Vector2d p9;//センター
  p9.x()=0;p9.y()=0;

  Eigen::Vector2d p10,p11;//オートリプレースメント
  p10.x()=0.5;p10.y()=2.5;
  p11.x()=3;p11.y()=3.2;

  Eigen::Vector2d p12;//ボールアプローチ
  p12.x()=1;p12.y()=1;

  cost_mat=MapCheck::makeFieldCostMat(size_x,size_y,space_value, cell_size);
  MapCheck::setInsideRectangle(&cost_mat,obstacle_value,p1,p2,p3,p4);
  MapCheck::setOutsideRectangle(&cost_mat,obstacle_value,p5,p6,p7,p8);
  MapCheck::setInsideCircle(&cost_mat,obstacle_value,p9,0.5);
  MapCheck::setInsideLine(&cost_mat,obstacle_value,p10,p11,0.2);
  MapCheck::setInsideOsiri(&cost_mat,obstacle_value,p12,-45,0.1);
  //cv::Mat cost_mat = (cv::Mat_<int>(3,4) << 10,20,30,40,50,60,50,40,30,20,10,0);
  //cv::Mat img = cv::imread("/home/adachi/Lena_gray.jpg");
  
  //cv::Mat cost_mat;
  //cv::cvtColor(img, cost_mat,cv::COLOR_BGR2GRAY);
  //cv::imshow("opencv_logo", cost_mat);
  //std::cout<<cost_mat<<std::endl;
  this->pub_map->publish(MapCheck::getMap(cost_mat,cell_size));
    
}
