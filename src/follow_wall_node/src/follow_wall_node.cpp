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
float err_I_, err_prev_, err_D_;
bool err_prev_available{false};

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

  // stop moving
  msg_cmdVel.linear.x = 0.0;
  msg_cmdVel.angular.z = 0.0;
  pub_cmdVel.publish(msg_cmdVel);
  ros::spinOnce();
  loopRate.sleep();
  // shutdown node
  ros::shutdown();

  return 0;
}

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
  const float theta = 45.0 / 180 * 3.14159;
  float INF_THD{1.05};      // [m] max distance to consider as inf
  float lookAheadDist{0.3}; // [m] look-ahead distance //0.2
  float crossTrackErr{0.0};
  float heading{0};
  float ref_frontRay; // reference value if the wall is perfectly straight and
                      // there is no obstacle ahead

  // bool isRightInf = rightRay > INF_THD;
  // bool isFrontInf = frontRay > INF_THD;
  // bool isRearInf = rearRay > INF_THD;

  bool isFrontInf = isinf(frontRay) || isnan(frontRay);
  bool isFrRightInf = isinf(frRightRay) || isnan(frRightRay);
  bool isRightInf = isinf(rightRay) || isnan(rightRay);

  // using 3 params for motion control
  // cross-track error, heading angle, frontRay
  // calculate heading angle
  if (isRightInf || isFrRightInf) {
    heading = 0.0; //-10.0 / 180 * 3.14159;
  } else {
    heading = std::atan((cos(theta) - rightRay / frRightRay) / sin(theta));
  }

  // calculate frontRay
  if (isFrontInf) {
    frontRay = 0.0;
  }

  /*if (isRightInf || isFrRightInf) {
    ref_frontRay = 0.0;
  } else {
    if (heading >
        -1.0 / 180 * 3.14159) {      // robot is almost parallel to the wall
      ref_frontRay = 3 * frRightRay; // or is heading faraway from the wall
    } else {
      ref_frontRay = (rightRay * cos(heading)) / cos(heading + 3.14159 / 2);
    }
  }*/
  ref_frontRay = 0.4;

  // calculate cross-track error
  if (isRightInf || isFrRightInf) {
    crossTrackErr = 0.0;
  } else {
    crossTrackErr = rightRay * cos(heading);
    crossTrackErr +=
        lookAheadDist * sin(heading); // ajusted with look ahead distance
  }

  // 1st method to deal with ortho corner
  //   if (frontRay < ref_frontRay) {
  //     crossTrackErr = 0.0;
  //   }

  //---***

  //---***
  // PID with kD = 0, kI =0
  // cross-track error, heading angle, frontRay
  const float DESIRE_crossTrackErr = 0.25;
  const float kP_crossTrackErr = 0.75;
  const float kI_crossTrackErr = 0.005;
  const float kD_crossTrackErr = 0.025;
  const float MAX_linX = 0.075;
  const float MAX_angZ = 0.3;
  float err_, curvature_K;

  err_ = DESIRE_crossTrackErr - crossTrackErr;
  err_I_ += err_;

  if (err_prev_available) {
    err_D_ = err_ - err_prev_;
  } else {
    err_D_ = 0.0;
    err_prev_available = true;
  }

  err_prev_ = err_;

  // calculate cmd_angZ and cmd_linX
  //
  // ---1st method : err_ ->> steering ->> K = 1/R curvature of turning
  // **we control the turning curvature directly
  //   curvature_K = kP_crossTrackErr * err_;

  // ---2nd method : ft. pure-pursuit model
  // err_ ->> steering ->> K = 1/R curvature of turning
  // **we control the steering, then calculate turning curvature
  const float wheel_base = 0.2; // 0.15
  float steering;

  steering = kP_crossTrackErr * err_ + kI_crossTrackErr * err_I_ +
             kD_crossTrackErr * err_D_;
  curvature_K = tan(steering) / wheel_base;

  // two-wheels robot model
  // linear_speed = wheel_curvature * R * avg_angular_speed
  // R = 1/K : wheel_curvature stand for 1/wheel_radius
  // linear_speed = wheel_radius * sum of (left/right rotation speed)
  // avg_angular_speed = average of (left/right rotation speed)

  // Move the robot around:
  // Be careful to NOT exceed a linear speed of 0.19 and angular of 0.49
  // because, otherwise, the node will be terminated for security reasons.
  // linear 0.01 0.02 .. 0.05 .. 0.1 maxi
  // angular 0.1 .. 0.3 maxi

  // 2nd method to deal with ortho corner
  if (frontRay < ref_frontRay) {
    curvature_K = 1.0 / (ref_frontRay - DESIRE_crossTrackErr);
  }

  //*set linear speed depending on turning curvature
  if (abs(curvature_K) < 1 / 1.5) {
    cmd_linX = MAX_linX; // 0.05 0.1
  } else if (abs(curvature_K) < 1 / 0.4) {
    cmd_linX = 0.05; // 0.03 0.05
  } else if (abs(curvature_K) < 1 / 0.3) {
    cmd_linX = 0.01; // 0.01
  } else if (abs(curvature_K) < 1 / 0.2) {
    cmd_linX = 0.005; // 0.005
  } else {
    cmd_linX = 0.02;
  }
  // WHEEL_RADIUS 0.03294510105
  // https://github.com/ROBOTIS-GIT/OpenCR/issues/225
  // The default value of the circumference of burger is 0.207
  const float wheel_curvature = 1 / 0.0329; // 1/wheel_radius
  const float width_axle = 0.16;            // distance betwwen two wheels
  cmd_angZ = wheel_curvature * curvature_K * width_axle * cmd_linX;

  // cmd_angZ saturation
  if (abs(cmd_angZ) > MAX_angZ) {
    cmd_angZ = (cmd_angZ > 0 ? MAX_angZ : -MAX_angZ);
  } // max cmd_angZ

  if (abs(cmd_angZ) < 0.01) {
    cmd_angZ = 0.0;
  } // min cmd_angZ

  //---------------------------
  // ROS_INFO("isFrontInf %s", isFrontInf ? "true" : "false");
  // ROS_INFO("rightRay %f --- frontRay %f -- frRightRay %f", rightRay,
  // frontRay,
  // frRightRay);

  ROS_INFO("   1    crossTrackErr %f --- heading %f ", crossTrackErr, heading);
  ROS_INFO("    2   err_ %f                         ", err_);
  ROS_INFO("     3  curvature_K %f                  ", curvature_K);
  ROS_INFO("      4 cmd_angZ %f --- cmd_linX %f     ", cmd_angZ, cmd_linX);

  // controller tuning -------------------------------------------------
  // kP_crossTrackErr   mid impact   0.5 x0.75 *0.95
  // kI_crossTrackErr   mid impact   0.01 0.0 *0.005 0.0025
  // kD_crossTrackErr   mid impact   *0.025
  // lookAheadDist                   0.2 *0.3
  // ref_frontRay                    0.3 *0.4
  // when (frontRay < ref_frontRay)  *method2
  // min cmd_angZ                    *0.01
  // max cmd_linX                    0.05 0.1 0.075

  // modeling tuning ---------------------------------------------------
  // wheel_base         mid impact   0.1 0.15 *0.2
}