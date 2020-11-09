/**
 * @file myTest.cpp
 * @author Govind Ajith Kumar
 * @copyright MIT License
 * @brief Tests to test the talker node using g-test framework
 */

/**
 *MIT License
 *Copyright (c) 2020 Govind Ajith Kumar
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeStringService.h"
#include "std_msgs/String.h"

std::shared_ptr<ros::NodeHandle> nh;

/**
 * @brief     Tests whether the changeBaseString service can modify the output message
 * @param     testTalkerNode       talker node
 * @param     stringChange        string to be changed to
 */

TEST(testTalker, stringChange) {
  ros::NodeHandle n;

  auto client = n.serviceClient
  <beginner_tutorials::changeStringService>("changeStringService");

// setting service object
  beginner_tutorials::changeStringService srv;

// new string
  srv.request.inString = "changeStringService";

// calls the service
  client.call(srv.request, srv.response);

// checks equality of string
  EXPECT_STREQ("changeStringService", srv.response.outString.c_str());
}

/**
 * @brief      Main Function
 *
 * @param[in]  argc  
 * @param      argv  
 *
 * @return     0 in default case
 */

int main(int argc,
         char **argv) {
  ros::init(argc, argv, "myTest");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

