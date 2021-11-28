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
 * @file talker.hpp
 * @author Sakshi Kakde
 * @brief A class to publish string data on a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef INCLUDE_TURTLEBOT3_WALKER_WALKER_HPP_
#define INCLUDE_TURTLEBOT3_WALKER_WALKER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>


class Walker {
 public:
    /**
     * @brief default constructor for Talker object
     * 
     */
    Walker();
    /**
     * @brief explicit constructor for Talker object
     * 
     * @param nh_p pointer to nodehandle
     */
    explicit Walker(ros::NodeHandle* nh_p);
    /**
     * @brief Destroy the Talker object
     * 
     */
    ~Walker();
    /**
     * @brief Runs the ros::ok loop and publishes data at a predefined rate
     * 
     */
    void runNode();
    ros::NodeHandle* nh_p;  // nodehandle

 private:
    std::string publisher_topic_name,
                subscriber_topic_name;  // ROS publisher topic name
    int scan_range_param;
    float distance_threshold;
    ros::Publisher cmd_vel_pub;  // ROS publisher object
    int publisher_rate;  // rate of publishing
    /**
     * @brief Function to init params
     * 
     */
    ros::Subscriber scan_sub;  // ROS Subscriber object
    void initParams();
    /**
     * @brief Function to init publishers
     * 
     */
    void initPublishers();
    /**
     * @brief Function to init subscribers
     * 
     */
    void initSubscriber();

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    geometry_msgs::Twist processScan(const sensor_msgs::LaserScan& scan);
};
#endif  // INCLUDE_TURTLEBOT3_WALKER_WALKER_HPP_
