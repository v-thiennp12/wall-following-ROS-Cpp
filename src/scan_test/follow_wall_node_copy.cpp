#include "find_wall_node/FindWallMessage.h"
#include "odom_record_node/OdomRecordMsgAction.h"
#include "odom_record_node/OdomRecordMsgGoal.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "ros/time.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iterator>
#include <math.h>
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

  // odom record action client
  odom_record_node::OdomRecordMsgGoal msg_odomActionGoal;
  actionlib::SimpleActionClient<odom_record_node::OdomRecordMsgAction>
      odomRecordClient("odom_record_as", true);
  odomRecordClient.waitForServer();
  odomRecordClient.sendGoal(msg_odomActionGoal);
  actionlib::SimpleClientGoalState state_result = odomRecordClient.getState();
  // int PENDING = 0;
  // int ACTIVE = 1;
  // int DONE = 2;
  // int WARN = 3;
  // int ERROR = 4;

  // find wall service
  // msg_FindWallMessage.request.good2go = true;
  ros::service::waitForService("/find_wall_server");
  srv_findWall = nodeHdl.serviceClient<find_wall_node::FindWallMessage>(
      "/find_wall_server");
  srv_findWall.call(msg_FindWallMessage);

  // follow wall
  ros::Subscriber sub_laserScan;
  ros::Publisher pub_cmdVel;
  ros::Rate loopRate(10);

  sub_laserScan = nodeHdl.subscribe("/scan", 1000, callbackLaserScan);
  pub_cmdVel = nodeHdl.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  geometry_msgs::Twist msg_cmdVel;

  while (ros::ok() &&
         (state_result == actionlib::SimpleClientGoalState::ACTIVE ||
          state_result == actionlib::SimpleClientGoalState::PENDING)) {
    msg_cmdVel.linear.x = cmd_linX;
    msg_cmdVel.angular.z = cmd_angZ;
    pub_cmdVel.publish(msg_cmdVel);
    // ROS_INFO("LinearX AngularZ %f %f ", cmd_linX, cmd_angZ);
    ros::spinOnce();
    loopRate.sleep();
    state_result = odomRecordClient.getState();
  }
  return 0;
}

// Move the robot around:
//  Be careful to NOT exceed a linear speed of 0.19 and angular of 0.49 because,
//  otherwise, the node will be terminated for security reasons.
// linear 0.01 0.02 .. 0.05 .. 0.1 maxi
// angular 0.1 .. 0.3 maxi

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg) {
  cmd_linX = 0;
  cmd_angZ = 0;
  // check laser scan message
  /*ROS_INFO("/scan.ranges length %f ",
           (msg->angle_max - msg->angle_min) / msg->angle_increment + 1);
           */
  float rightRay = msg->ranges[180];   // -y axis
  float frontRay = msg->ranges[360];   // +x axis
  float frRightRay = msg->ranges[270]; // +45° from front ray
  float INF_THD{1.05};                 // [m] max distance to consider as inf
  float lookAheadDist{0.05};           // [m] look-ahead distance
  float crossTrackErr{0.0};
  float heading{0};

  // bool isRightInf = rightRay > INF_THD;
  // bool isFrontInf = frontRay > INF_THD;
  // bool isRearInf = rearRay > INF_THD;

  bool isFrontInf = isinf(frontRay) || isnan(frontRay);
  bool isFrRightInf = isinf(frRightRay) || isnan(frRightRay);
  bool isRightInf = isinf(rightRay) || isnan(rightRay);

  // calculate cross-track error and heading angle
  if (isRightInf && !(frontRay > 0.3)) {
    // srv_findWall.call(msg_FindWallMessage);
    crossTrackErr = 0.0;
  }

  if (isRightInf && frontRay > 0.3) {
    // srv_findWall.call(msg_FindWallMessage);
    // heading = 0.0;
    heading = +10.0 / 180 * 3.14159;
    crossTrackErr = 0.25;
  }

  if (!isRearInf && !isRightInf) {
    crossTrackErr =
        rightRay * rearRay / std::sqrt(rightRay * rightRay + rearRay * rearRay);
    heading = -std::acos(crossTrackErr / rightRay);
  }

  if (!isFrontInf && !isRightInf) {
    crossTrackErr = rightRay * frontRay /
                    std::sqrt(rightRay * rightRay + frontRay * frontRay);
    heading = +std::acos(crossTrackErr / rightRay);
  }

  if (isFrontInf && isRearInf && !isRightInf) {
    crossTrackErr = rightRay;
    heading = 0.0;
  }

  if (isFrontInf && isRearInf && isRightInf) {
    crossTrackErr = 0.0;
    heading = 0.0;
  }
  // ------------------------------

  // if crossTrackErr < 0.2
  if (crossTrackErr > 0.5) {
    cmd_linX = 0.05;
    cmd_angZ = +0.0;
  }

  if ((crossTrackErr > 0.3) && (crossTrackErr < 0.5)) {
    cmd_linX = 0.05;
    cmd_angZ = -0.05;
  }
  //---------------------------

  // if crossTrackErr < 0.2
  if ((crossTrackErr < 0.2) && (frontRay > 0.3)) {
    cmd_linX = 0.05; // 0.05
    cmd_angZ = +0.1; // 0.2
  }

  if ((crossTrackErr < 0.2) && !(frontRay > 0.3)) {
    cmd_linX = 0.0;  // 0.05
    cmd_angZ = +0.1; // 0.2
  }
  //---------------------------

  // if 0.2 < crossTrackErr < 0.5
  if ((crossTrackErr > 0.2) && (crossTrackErr < 0.5) &&
      (std::abs(heading) < 10.0 / 180 * 3.14159)) {
    cmd_linX = 0.05;
    cmd_angZ = 0.0;
  }

  if ((crossTrackErr > 0.2) && (crossTrackErr < 0.5) &&
      (heading < -5.0 / 180 * 3.14159)) {
    cmd_linX = 0.0;  // 0.05
    cmd_angZ = -0.1; // 0.1
  }

  if ((crossTrackErr > 0.2) && (crossTrackErr < 0.3) &&
      (heading > +5.0 / 180 * 3.14159)) {
    cmd_linX = 0.0;  // 0.05
    cmd_angZ = +0.1; // 0.1
  }

  if (frontRay < 0.3) {
    // srv_findWall.call(msg_FindWallMessage);
    cmd_linX = 0.0;
    cmd_angZ = +0.1;
    crossTrackErr = frontRay;
  }
  //---------------------------

  ROS_INFO("rightRay %f --- frontRay %f -- rearRay %f", rightRay, frontRay,
           rearRay);
  ROS_INFO("         crossTrackErr %f --- heading %f ", crossTrackErr, heading);
  ROS_INFO("isFrontInf %s", isFrontInf ? "true" : "false");
}