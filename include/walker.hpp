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

/** @file walker.hpp
 *  @brief Definition of class Walker
 *
 *  This file contains definitions of class Walker which subscribes to
 *  /scan topic and uses it to determine if robot is close to obstacle
 *
 *  @author Huei Tzu Tsai
 *  @date   04/13/2017
*/

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/**
 *  @brief Class definition of Walker node
*/
class Walker {
 public:
     /**
      *   @brief  Constructor of Walker class
      *
      *   @param  none
      *   @return none
     */
     Walker() : bClose(false) {}

     /**
      *   @brief  Deconstructor of Walker class
      *
      *   @param  none
      *   @return none
     */
     ~Walker() {}

     /**
      *   @brief  Callback function of Walker which subscribes
      *           to topic /scan to determine if robot is close to obstacle
      *
      *   @param  pointer to scan info of type sensor_msgs::LaserScan
      *   @return none
     */
     void sensorCallback(
         const sensor_msgs::LaserScan::ConstPtr&);


     /**
      *   @brief  Check if robot is close to obstacle
      *
      *   @param  none
      *   @return true if robot is within 0.5m to obstacle \n
      *           false otherwise
     */
     bool isClose(void) { return bClose; }

 private:
     bool bClose;            ///< flag to record whether robot is
                             ///< within 0.5m to obstacle
};

#endif  // INCLUDE_WALKER_HPP_
