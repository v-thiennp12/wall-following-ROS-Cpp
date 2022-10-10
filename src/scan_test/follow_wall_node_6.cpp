#include "find_wall_node/FindWallMessage.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <iterator>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

using namespace std;

float cmd_linX, cmd_angZ;
find_wall_node::FindWallMessage msg_FindWallMessage;
ros::ServiceClient srv_findWall;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);

int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_wall_node");
  ros::NodeHandle nodeHdl;

  // find wall
  // msg_FindWallMessage.request.good2go = true;
  ros::service::waitForService("/find_wall_server");
  srv_findWall = nodeHdl.serviceClient<find_wall_node::FindWallMessage>(
      "/find_wall_server");
  srv_findWall.call(msg_FindWallMessage);

  // follow wall
  ros::Subscriber sub_laserScan;
  ros::Publisher pub_cmdVel;
  ros::Rate loopRate(5);

  sub_laserScan = nodeHdl.subscribe("/scan", 1000, callbackLaserScan);
  pub_cmdVel = nodeHdl.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  geometry_msgs::Twist msg_cmdVel;

  while (ros::ok()) {
    msg_cmdVel.linear.x = cmd_linX;
    msg_cmdVel.angular.z = cmd_angZ;
    pub_cmdVel.publish(msg_cmdVel);
    // ROS_INFO("LinearX AngularZ %f %f ", cmd_linX, cmd_angZ);
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

// Move the robot around:
//  Be careful to NOT exceed a linear speed of 0.19 and angular of 0.49 because,
//  otherwise, the node will be terminated for security reasons.

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg) {
  cmd_linX = 0;
  cmd_angZ = 0;
  // check laser scan message
  /*ROS_INFO("/scan.ranges length %f ",
           (msg->angle_max - msg->angle_min) / msg->angle_increment + 1);
           */
  float rightRay = msg->ranges[180];
  float frontRay = msg->ranges[360];
  bool goStraight = true;

  if ((rightRay > 0.3) && (rightRay < 0.5)) {
    cmd_linX = 0.05;
    cmd_angZ = -0.05;
    goStraight = false;
  }

  if (rightRay < 0.2) {
    cmd_linX = 0.05;
    cmd_angZ = +0.05;
    goStraight = false;
  }

  if ((rightRay > 0.2) && (rightRay < 0.3)) {
    cmd_linX = 0.05;
    cmd_angZ = 0.0;
  }

  if ((rightRay > 0.2) && (rightRay < 0.3) && (frontRay < 0.3)) {
    cmd_linX = 0.05;
    cmd_angZ = +0.1;
    goStraight = false;
  }

  if ((frontRay < 0.2)) {
    // srv_findWall.call(msg_FindWallMessage);
    cmd_linX = -0.1;
    cmd_angZ = 0.0;
    goStraight = false;
  }

  if (goStraight) {
    cmd_linX = 0.1;
    cmd_angZ = 0.0;
  }
}