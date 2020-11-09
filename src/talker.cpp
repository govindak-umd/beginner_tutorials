/**
 * @file talker.cpp
 * @author Govind Ajith Kumar
 * @copyright MIT License
 * @brief Implementing the publisher
 * This is the talker file for ROS subscriber-publisher example.
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
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeStringService.h"
#include "../include/talker.h"

DefaultMessage default_message;

/**
 * @brief     Callback for the service
 *
 * @param      req   For requesting the data sent to the service
 * @param      res   For responding to the service
 *
 * @return     a boolean type of value is returned
 */
bool newMessage(beginner_tutorials::changeStringService::Request &req,
                beginner_tutorials::changeStringService::Response &res) {
  default_message.output = req.inString;
  ROS_WARN_STREAM("USER INPUT RECEIVED: STRING CHANGED!");
  res.outString = req.inString;
  return true;
}

/**
 * @brief  Broadcaster Function to broadcast tf frame
 * @param  none
 * @return none
 */
void tfBroadcaster(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(cos(ros::Time::now().toSec()),
                                    sin(ros::Time::now().toSec()), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,
                                          ros::Time::now(), "world", "talk"));
}


/**
* This tutorial demonstrates simple sending of messages over the ROS system.
*/
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  ros::Publisher chatter_pub = n.advertise < std_msgs::String
      > ("chatter", 1000);

  auto server = n.advertiseService("changeStringService", newMessage);

  /**
   * The frequency at which talker publishes into the topic
   */
  double my_frequency;

  /**
   * getting the frequency from the topic
   */
  n.getParam("/freq", my_frequency);

  /**
   * Outputs varied based on the frequency or rate set by the user 
   * in the launch file
   */
  if (my_frequency <= 500 && my_frequency > 0) {
    ROS_DEBUG_STREAM("RATE OF PRINTING : " << my_frequency);
  } else if (my_frequency > 500) {
    ROS_ERROR_STREAM("Very Fast. Slow down! ");
    ROS_WARN_STREAM("Dialing it down back to 100 Hz");
    my_frequency = 100;
  } else if (my_frequency < 0) {
    ROS_ERROR_STREAM("Loop Rate has to be greater than 0");
    ROS_WARN_STREAM("Reverting back to a positive minimum loop frequency");
    my_frequency = 5;
  } else if (my_frequency == 0) {
    ROS_FATAL_STREAM("Loop rate can't be zero. So, please change this");
    ROS_WARN_STREAM("Reverting back to a positive minimum loop frequency");
    my_frequency = 5;
  }


  ros::Rate loop_rate(my_frequency);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
    */

  while (ros::ok()) {

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    std_msgs::String msg;


    msg.data = default_message.output;



    ROS_INFO("%s", msg.data.c_str());


    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    chatter_pub.publish(msg);


    // Calling the Broadcaster Function to broadcast a tf frame called /talk with parent /world
    tfBroadcaster();


    ros::spinOnce();


    loop_rate.sleep();

  }

  return 0;
}

