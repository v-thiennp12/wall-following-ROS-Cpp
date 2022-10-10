#include "odom_record_node/OdomRecordMsgAction.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>

class RecordOdomAction {
protected:
  // node handle, must be here before other thing
  ros::NodeHandle nodeHdl;

  // topics subscriber
  ros::Subscriber odomSub;

  //   odom_record_node::OdomRecordMsgGoal goalMsg;
  odom_record_node::OdomRecordMsgFeedback feedbackMsg;
  odom_record_node::OdomRecordMsgResult resultMsg;
  std::string action_name_;

  // variable
  ros::Rate *rate_;
  int rate_hz_ = 1;
  //   ros::Rate subRate(2);
  bool success = false;
  nav_msgs::Odometry odomMsg;
  //   bool good2go;
  float stepDist;
  int stepCount{0};

public:
  // action server
  actionlib::SimpleActionServer<odom_record_node::OdomRecordMsgAction>
      actionServer;

  RecordOdomAction(std::string name)
      : actionServer(nodeHdl, name, false), action_name_(name) {

    // register callbacks
    actionServer.registerGoalCallback(
        boost::bind(&RecordOdomAction::callbackGoal, this));
    actionServer.registerPreemptCallback(
        boost::bind(&RecordOdomAction::callbackPreempt, this));
    actionServer.start();
    // topics
    odomSub =
        nodeHdl.subscribe("/odom", 1, &RecordOdomAction::callbackOdomSub, this);

    rate_ = new ros::Rate(rate_hz_);
  };

  ~RecordOdomAction(void){};

  void callbackOdomSub(const nav_msgs::OdometryConstPtr &msg) {
    odomMsg = *msg;
    // make sure that the action hasn't been canceled
    // if (!as_.isActive())
    // return;

    if (actionServer.isActive()) { // check if action server on-air
      // ROS_INFO("callbackOdomSub");

      if (!actionServer.isPreemptRequested() && ros::ok()) {
        // increment step count
        // stepCount = length(resultMsg.list_of_odoms)
        stepCount += 1;

        // ROS_INFO("callbackOdomSub - stepCount %d", stepCount);
        //  feedbackMsg.current_total += 1;
        //  direct without pointer
        //  resultMsg.list_of_odoms.push_back(msg->pose.pose.position);
        resultMsg.list_of_odoms.push_back(
            odomMsg.pose.pose.position); // copy to odomMsg

        stepDist = 0.0;
        if (stepCount > 1) {
          stepDist += std::pow(resultMsg.list_of_odoms[stepCount - 1].x -
                                   resultMsg.list_of_odoms[stepCount - 2].x,
                               2);
          stepDist += std::pow(resultMsg.list_of_odoms[stepCount - 1].y -
                                   resultMsg.list_of_odoms[stepCount - 2].y,
                               2);
          // stepDist += std::pow(resultMsg.list_of_odoms[stepCount].z -
          // resultMsg.list_of_odoms[stepCount - 1].z, 2);
          stepDist = std::sqrt(stepDist);
        }

        feedbackMsg.current_total += stepDist;
        actionServer.publishFeedback(feedbackMsg);
        // ROS_INFO("%f: feedbackMsg.current_total", feedbackMsg.current_total);

        // if (stepCount > 15) {
        // success = true;
        //}

        // 5 m minimum for a loop
        if (feedbackMsg.current_total > 5) {
          if (stepCount > 1) {
            float dist_ = 0.0;
            dist_ += std::pow(resultMsg.list_of_odoms[stepCount - 1].x -
                                  resultMsg.list_of_odoms[0].x,
                              2);
            dist_ += std::pow(resultMsg.list_of_odoms[stepCount - 1].y -
                                  resultMsg.list_of_odoms[0].y,
                              2);
            // dist_ = std::sqrt(dist_);
            if (dist_ < 0.2 * 0.2) { // loop-closure checking
              success = true;
            }
          }
        }

        if (success) {
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          actionServer.setSucceeded(resultMsg);
          resultMsg.list_of_odoms.clear();
        };

        // not work
        //   usleep(1000);
        //   ros::spinOnce(); //not work

        // not work 2
        //   subRate.sleep();

        // that worked !!! 3
        rate_->sleep();
      }
    }
    // } else {
    // } set aborted
    //  as_.setAborted(result_);
  };

  void callbackGoal() {
    actionServer.acceptNewGoal();
    // good2go = actionServer.acceptNewGoal()->good2go;
    stepCount = 0;
    stepDist = 0.0;
    feedbackMsg.current_total = 0;
    success = false;
    resultMsg.list_of_odoms.clear();

    ROS_INFO("%d callbackGoal", stepCount);
  };

  void callbackPreempt() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    actionServer.setPreempted(resultMsg);
    resultMsg.list_of_odoms.clear();
  };
};

// user:~/catkin_ws$ rosmsg show Odometry
// [nav_msgs/Odometry]:
// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// string child_frame_id
// geometry_msgs/PoseWithCovariance pose
//   geometry_msgs/Pose pose
//     geometry_msgs/Point position
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Quaternion orientation
//       float64 x
//       float64 y
//       float64 z
//       float64 w
//   float64[36] covariance
// geometry_msgs/TwistWithCovariance twist
//   geometry_msgs/Twist twist
//     geometry_msgs/Vector3 linear
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Vector3 angular
//       float64 x
//       float64 y
//       float64 z
//   float64[36] covariance
