#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include "beginner_tutorials/changeStringService.h"
#include "std_msgs/String.h"

TEST(talkerTestingNode, testExistence) {

ros::NodeHandle n;

auto client = n.serviceClient
        <beginner_tutorials::changeStringService>("changeStringService");

EXPECT_TRUE(client.waitForExistence(ros::Duration(5)));
}


TEST(testTalker, testModifyMessage) {
ros::NodeHandle n;

auto client = n.serviceClient
        <beginner_tutorials::changeStringService>("changeStringService");

beginner_tutorials::changeStringService srv;

srv.request.inString = "changeStringService";

client.call(srv.request, srv.response);

EXPECT_STREQ("changeStringService", srv.response.outString.c_str());
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "changeStringService");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

