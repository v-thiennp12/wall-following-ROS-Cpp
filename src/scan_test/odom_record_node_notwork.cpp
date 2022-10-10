#include "odom_record_node/OdomRecordMsgAction.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

void callbackOdomSub(const nav_msgs::OdometryConstPtr &msg);
void callbackAction(const odom_record_node::OdomRecordMsgGoalConstPtr &goalMsg);
//   odom_record_node::OdomRecordMsgGoal goalMsg;
odom_record_node::OdomRecordMsgFeedback feedbackMsg;
odom_record_node::OdomRecordMsgResult resultMsg;
std::string ActionTopicName;
nav_msgs::Odometry odomMsg;

std::string name = "odom_record_as";
// action server
actionlib::SimpleActionServer<odom_record_node::OdomRecordMsgAction>
actionServer();

// variable
ros::Rate *rate_;
int rate_hz_ = 1;
bool success = false;

int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_record_as");
  ros::NodeHandle nodeHdl;
  actionlib::SimpleActionServer<odom_record_node::OdomRecordMsgAction>
      actionServer(nodeHdl, name, callbackAction, false);
  //   // topics
  ros::Subscriber odomSub = nodeHdl.subscribe("/odom", 1000, callbackOdomSub);
  actionServer.start();
  rate_ = new ros::Rate(rate_hz_);
  success = true;
  ros::spin();
  return 0;
}

void callbackOdomSub(const nav_msgs::OdometryConstPtr &msg) {
  odomMsg = *msg;
  ROS_INFO("callbackOdomSub");
}

void callbackAction(
    const odom_record_node::OdomRecordMsgGoalConstPtr &goalMsg) {
  ROS_INFO("%f: feedbackMsg.current_total", feedbackMsg.current_total);

  while (true) {
    if (actionServer.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", ActionTopicName.c_str());
      // set the action state to preempted
      actionServer.setPreempted();
      success = false;
      break;
    } else {
      feedbackMsg.current_total += 1;
      resultMsg.list_of_odoms.push_back(odomMsg.pose.pose.position);
    }

    actionServer.publishFeedback(feedbackMsg);
    rate_->sleep();
    ros::spinOnce();
  }
  if (success) {
    actionServer.setSucceeded(resultMsg);
  };
};