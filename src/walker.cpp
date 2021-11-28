/**
 * MIT License
 *
 * Copyright (c) 2021 Sakshi Kakde
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @file talker.cpp
 * @author Sakshi Kakde
 * @brief A class to publish string data on a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <turtlebot3_walker/walker.hpp>

Walker::Walker() {
}

Walker::Walker(ros::NodeHandle* nh_p) {
    this->nh_p = nh_p;
    obstacle_detector = std::make_unique<ObstacleDetector>();
    initParams();
    initPublishers();
    initSubscriber();
}

Walker::~Walker() {
      delete this->nh_p;
}

void Walker::initParams() {
    this->nh_p->param<std::string>("publisher_topic_name",
     this->publisher_topic_name, "/cmd_vel");
    this->nh_p->param<std::string>("subscriber_topic_name",
     this->subscriber_topic_name, "/scan");
    this->nh_p->param<int>("publisher_rate",
     this->publisher_rate, 10);
    this->nh_p->param<int>("scan_range",
     this->scan_range_param, 10);
    this->nh_p->param<float>("distance_threshold",
     this->distance_threshold, 1.0);
}

void Walker::initPublishers() {
  this->cmd_vel_pub = this->nh_p->advertise<geometry_msgs::Twist>(
        this->publisher_topic_name,
         this->publisher_rate, this);
}

void Walker::initSubscriber() {
  this->scan_sub = this->nh_p->subscribe(this->subscriber_topic_name,
     1, &Walker::scanCallback, this);
}

void Walker::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ROS_INFO_STREAM("Received scan");
  sensor_msgs::LaserScan scan = *msg;
  geometry_msgs::Twist vel = this->processScan(scan);
  ROS_INFO_STREAM("Publiching velocity");
  cmd_vel_pub.publish(vel);
}

geometry_msgs::Twist Walker::processScan(const sensor_msgs::LaserScan& scan) {
  geometry_msgs::Twist vel;
  bool obstacle_present = this->obstacle_detector->obstaclePresent(scan,
                          this->scan_range_param,
                          this->distance_threshold);
  // bool obstacle_present = false;
  // for (int i = 0; i < this->scan_range_param; i++) {
  //   if ((scan.ranges[i] <= this->distance_threshold)||
  //       (scan.ranges[359-i] <= this->distance_threshold)) {
  //     obstacle_present = true;
  //   }
  // }

  if (obstacle_present) {
      ROS_WARN_STREAM("OBSTACLE PRESENT");
      vel.linear.x = 0;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = 1;
  } else {
      vel.linear.x = 0.5;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = 0;
  }
  return vel;
}

void Walker::runNode() {
  ros::Rate loop_rate(this->publisher_rate);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Walker node is active");
    ros::spinOnce();
    loop_rate.sleep();
  }
}
