#include "RecordOdomAction.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "odom_record_as");
  RecordOdomAction odomRecorder("odom_record_as");
  ros::spin();
  return 0;
}

/* not work
//   ros::Rate subRate(1);
//   while (odomRecorder.actionServer.isActive() && ros::ok()) {
//     ros::spinOnce();
//     subRate.sleep();
//   } */