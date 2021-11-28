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
 * @brief A class to detect obstacle 
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef INCLUDE_TURTLEBOT3_WALKER_OBSTACLE_DETECTOR_HPP_
#define INCLUDE_TURTLEBOT3_WALKER_OBSTACLE_DETECTOR_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class ObstacleDetector {
 public:
    /**
     * @brief default constructor for Talker object
     * 
     */
    ObstacleDetector();
    /**
     * @brief Destroy the Talker object
     * 
     */
    ~ObstacleDetector();
    /**
     * @brief A function to detect obstacle
     * 
     * @param scan laser scan
     * @param scan_range Angular range to consider while detection
     * @param distance_threshold Max allowed distance for obstacle
     * @return true If obstacle present
     * @return false If obstacle not present
     */
    bool obstaclePresent(const sensor_msgs::LaserScan& scan,
                                       int scan_range,
                                       float distance_threshold);

 private:
};
#endif  // INCLUDE_TURTLEBOT3_WALKER_OBSTACLE_DETECTOR_HPP_
