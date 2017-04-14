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

/**
 *  @file main.cpp
 *  @brief Initial file of walker node
 *
 *  This file contains main entry point of a walker which drives a
 *  robot forward until it is close to an obstacle and spin in place
 *  until path is cleared.
 *
 *
 *
 *  @author Huei Tzu Tsai
 *  @date   04/13/2017
*/


#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "walker.hpp"


/*
 *   @brief  walker entrypoint
 *  
 *   @param  number of arguments
 *   @param  argument character array
 *   @return integer 0 upon exit success
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "walker");

    Walker walker;
    ros::NodeHandle n;

    // Subscribe topic /scan to receive laserscan info
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 50,
                                      &Walker::sensorCallback, &walker);

    // Register to publish topic velocity to drive robot forward
    // or spin in place until path is clear
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        geometry_msgs::Twist msg;

        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = 0.0;

        if (walker.isClose()) {
            // rotate about z-axis
            msg.angular.z = 1;
        } else {
            // move forward toward x-axis
            msg.linear.x = 0.1;
        }

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}

