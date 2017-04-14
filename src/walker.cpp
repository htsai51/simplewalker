/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2017 Huei-Tzu Tsai
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file walker.cpp
 *  @brief Implementation of class Walker methods
 *
 *  This file contains implemenation of callback function in node
 *  Walker.  Callback function receives messages published
 *  on topic /scan which contains Laserscan info that can be used to 
 *  determine the distance to obstacle
 *
 *  @author Huei Tzu Tsai
 *  @date   04/14/2017
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "walker.hpp"


void Walker::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for (int i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < 0.5) {
            bClose = true;
            ROS_DEBUG_STREAM("range " << msg->ranges[i] << " less than 0.5m");
            return;
        }
    }

    bClose = false;
    ROS_DEBUG_STREAM("range OK");
    return;
}



