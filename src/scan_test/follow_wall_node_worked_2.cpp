#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
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
void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
bool checkObjectCritical(const float &distance);
bool checkObjectNear(const float &distance);
bool checkObjectAway(const float &distance);

int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_wall_node");
  ros::NodeHandle nodeHdl;
  ros::Subscriber sub_laserScan;
  ros::Publisher pub_cmdVel;
  ros::Rate loopRate(20);

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
//  Be careful to NOT exceed a linear speed of 0.19 and angular of 0.01 because,
//  otherwise, the node will be terminated for security reasons.

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg) {
  //   cmd_linX = 0;
  //   cmd_angZ = 0;

  // check laser scan message
  /*ROS_INFO("/scan.ranges length %f ",
           (msg->angle_max - msg->angle_min) / msg->angle_increment + 1);
           */
  //   vector<float> frontRanges(begin(msg->ranges) + 300,
  //                             begin(msg->ranges) + 420); // 300 420
  //   vector<float> leftRanges(begin(msg->ranges) + 450,
  //                            begin(msg->ranges) + 630); // 450 630
  //   vector<float> rightRanges(begin(msg->ranges) + 90,
  //                             begin(msg->ranges) + 270); // 90 270
  //   vector<float> rearRanges(begin(msg->ranges), begin(msg->ranges) + 60); //
  //   0 60 rearRanges.insert(end(rearRanges), begin(msg->ranges) + 660,
  //                     end(msg->ranges));

  vector<float> frontRanges(begin(msg->ranges) + 345,
                            begin(msg->ranges) + 375); // 300 360 420
  vector<float> leftRanges(begin(msg->ranges) + 525,
                           begin(msg->ranges) + 555); // 450 540 630
  vector<float> rightRanges(begin(msg->ranges) + 165,
                            begin(msg->ranges) + 195); // 90 180 270
  vector<float> rearRanges(begin(msg->ranges), begin(msg->ranges) + 15); // 0 60
  rearRanges.insert(end(rearRanges), begin(msg->ranges) + 705,
                    end(msg->ranges));

  ROS_INFO("left i=540 - center i=360 - right i=180 ------ %f %f %f",
           msg->ranges[540], msg->ranges[360], msg->ranges[180]);

  ROS_INFO("%i", (int)frontRanges.size());
  ROS_INFO("%i", (int)rearRanges.size());
  ROS_INFO("%i", (int)leftRanges.size());
  ROS_INFO("%i", (int)rightRanges.size());

  // min_ind = std::min_element(frontRanges.begin(), frontRanges.end()) -
  // frontRanges.begin(); min_elem = *std::min_element(frontRanges.begin(),
  // frontRanges.end());

  float min_front = *std::min_element(frontRanges.begin(), frontRanges.end());
  float min_rear = *std::min_element(rearRanges.begin(), rearRanges.end());
  float min_left = *std::min_element(leftRanges.begin(), leftRanges.end());
  float min_right = *std::min_element(rightRanges.begin(), rightRanges.end());

  ROS_INFO("min_front %f", min_front);
  ROS_INFO("min_rear %f", min_rear);
  ROS_INFO("min_left %f", min_left);
  ROS_INFO("min_right %f", min_right);

  /* from -pi to +pi, 720 element inside ranges[] message
  (msg->angle_max - msg->angle_min) / msg->angle_increment + 1) = 720
   ->> at 0 degree (x axis) : i = 360
   ->> at -90 degree (y axis), right side : i = 180
   ->> at +90 degree , left side : i = 540
   rotation : different than this, cmd_angZ -0.2 for right turn, cmd_angZ +0.2
  for left turn */

  bool leftCritical = checkObjectCritical(min_left);
  bool leftNear = checkObjectNear(min_left);
  bool leftAway = checkObjectAway(min_left);

  bool rightCritical = checkObjectCritical(min_right);
  bool rightNear = checkObjectNear(min_right);
  bool rightAway = checkObjectAway(min_right);

  bool midCritical = checkObjectCritical(min_front);
  bool midNear = checkObjectNear(min_front);
  bool midAway = checkObjectAway(min_front);

  //   bool leftCritical = checkObjectCritical(msg->ranges[540]);
  //   bool leftNear = checkObjectNear(msg->ranges[540]);
  //   bool leftAway = checkObjectAway(msg->ranges[540]);

  //   bool rightCritical = checkObjectCritical(msg->ranges[180]);
  //   bool rightNear = checkObjectNear(msg->ranges[180]);
  //   bool rightAway = checkObjectAway(msg->ranges[180]);

  //   bool midCritical = checkObjectCritical(msg->ranges[360]);
  //   bool midNear = checkObjectNear(msg->ranges[360]);
  //   bool midAway = checkObjectAway(msg->ranges[360]);

  // If the distance to an obstacle in front of the robot is bigger than 1
  // meter, the robot will move forward

  // If the distance to an obstacle at the left side of the robot is smaller
  // than 0.3 meters, the robot will turn right
  if (leftNear) {
    cmd_linX = 0.05;
    cmd_angZ = +0.0;
  }
  // If the distance to an obstacle at the right side of the robot is smaller
  // than 0.3 meters, the robot will turn left
  if (rightNear) {
    cmd_linX = 0.05;
    cmd_angZ = +0.0;
  }

  if (leftCritical) {
    cmd_linX = 0.0;
    cmd_angZ = +0.1;
  }
  // If the distance to an obstacle at the right side of the robot is smaller
  // than 0.3 meters, the robot will turn left
  if (rightCritical) {
    cmd_linX = 0.0;
    cmd_angZ = +0.1;
  }

  if (midAway) {
    cmd_linX = 0.1;
    cmd_angZ = +0.05;
  }

  if (midNear) {
    cmd_linX = 0.0;
    cmd_angZ = +0.05;
  }

  // If the distance to an obstacle in front of the robot is smaller than 1
  // meter, the robot will turn left
  if (midCritical) {
    cmd_linX = -0.05;
    cmd_angZ = +0.0;
  }

  /*
bool leftCritical = checkObjectCritical(msg->ranges[540]);
bool leftNear = checkObjectNear(msg->ranges[540]);
bool leftAway = checkObjectAway(msg->ranges[540]);

bool rightCritical = checkObjectCritical(msg->ranges[180]);
bool rightNear = checkObjectNear(msg->ranges[180]);
bool rightAway = checkObjectAway(msg->ranges[180]);

bool midCritical = checkObjectCritical(msg->ranges[360]);
bool midNear = checkObjectNear(msg->ranges[360]);
bool midAway = checkObjectAway(msg->ranges[360]);

ROS_INFO("-----------------------------------------------------------");


if (leftAway) {
  cmd_linX = 0.05;
  cmd_angZ = +0.01;
  ROS_INFO("leftAway ===================================== turn left slow");
}

if (midNear && !leftAway) {
  cmd_linX = 0.05;
  cmd_angZ = -0.1;
  ROS_INFO("midNear && !leftAway  ========================= turn right");
}

if (midNear && !rightAway) {
  cmd_linX = 0.05;
  cmd_angZ = +0.1;
  ROS_INFO("midNear && !rightAway ========================= turn left");
}

if (midAway && rightNear) {
  cmd_linX = 0.05;
  cmd_angZ = 0;
  ROS_INFO("midAway && rightNear ========================== straight");
}

if (midAway && leftNear) {
  cmd_linX = 0.05;
  cmd_angZ = 0;
  ROS_INFO("midAway && leftNear  ========================== straight");
}

if (rightCritical || midCritical) {
  cmd_linX = 0.0;
  cmd_angZ = +0.2;
  ROS_INFO("rightCritical || midCritical=================== rotate left");
}

if (leftCritical) {
  cmd_linX = 0.0;
  cmd_angZ = -0.2;
  ROS_INFO("leftCritical  ================================= rotate right");
}
*/
  /*
bool leftCritical = checkObjectCritical(msg->ranges[540]);
bool leftNear = checkObjectNear(msg->ranges[540]);
bool leftAway = checkObjectAway(msg->ranges[540]);

bool rightCritical = checkObjectCritical(msg->ranges[180]);
bool rightNear = checkObjectNear(msg->ranges[180]);
bool rightAway = checkObjectAway(msg->ranges[180]);

bool midCritical = checkObjectCritical(msg->ranges[360]) ||
                   checkObjectCritical(msg->ranges[330]) ||
                   checkObjectCritical(msg->ranges[390]);
bool midNear = checkObjectNear(msg->ranges[330]) ||
               checkObjectNear(msg->ranges[300]) ||
               checkObjectNear(msg->ranges[390]);
bool midAway = checkObjectAway(msg->ranges[360]) &&
               checkObjectAway(msg->ranges[330]) &&
               checkObjectAway(msg->ranges[390]);

ROS_INFO("-----------------------------------------------------------");

if (midNear && !rightNear && cmd_angZ <= 0) {
  cmd_linX = 0.05;
  cmd_angZ = -0.1;
  ROS_INFO("midNear && !rightNear ==================== turn right slow");
}

if (midNear && !leftNear && cmd_angZ >= 0) {
  cmd_linX = 0.05;
  cmd_angZ = +0.1;
  ROS_INFO("midNear && !leftNear ===================== turn left slow");
}

if (leftAway && rightAway && midAway) {
  cmd_linX = 0.05;
  cmd_angZ = 0;
  ROS_INFO("leftAway && rightAway && midAway =============== straight");
}

if ((leftNear || rightNear) && midAway) {
  cmd_linX = 0.05;
  cmd_angZ = 0;
  ROS_INFO("(leftNear || rightNear) && midAway ======= straight");
}

if (leftNear && midNear && cmd_angZ <= 0) {
  cmd_linX = 0.05;
  cmd_angZ = -0.1;
  ROS_INFO("leftNear && midNear ====================== turn right");
}

if (rightNear && midNear && cmd_angZ >= 0) {
  cmd_linX = 0.05;
  cmd_angZ = +0.1;
  ROS_INFO("rightNear && midNear ===================== turn left");
}

if ((midCritical || rightCritical) && cmd_angZ >= 0) {
  cmd_linX = -0.05;
  cmd_angZ = +0.1;
  ROS_INFO("midCritical || rightCritical =================== rotate left");
}

if (leftCritical && cmd_angZ <= 0) { // !midCritical && !rightCritical &&
  cmd_linX = -0.05;
  cmd_angZ = -0.1;
  ROS_INFO("!midCritical && !rightCritical && leftCritical = rotate right");
} */
}

bool checkObjectCritical(const float &distance) {
  if (distance < 0.3) {
    return true;
  } else {
    return false;
  }
}

bool checkObjectNear(const float &distance) {
  // if ((distance > 0.2) && (distance < 0.3)) {
  if ((distance < 0.5)) {
    return true;
  } else {
    return false;
  }
}

bool checkObjectAway(const float &distance) {
  if (distance > 0.5) {
    return true;
  } else {
    return false;
  }
}

/*
user:~/catkin_ws$ rostopic echo /scan -n1
header:
  seq: 306
  stamp:
    secs: 1065
    nsecs: 919000000
  frame_id: "base_scan"
angle_min: -3.1415998935699463
angle_max: 3.1415998935699463
angle_increment: 0.008738803677260876
time_increment: 0.0
scan_time: 0.0
range_min: 0.11999999731779099
range_max: 3.5
ranges:
[1.1033803224563599, 1.0879896879196167, 1.0625933408737183, 1.0773626565933228, 1.0696614980697632,
1.0656236410140991, 1.0651580095291138, 1.9883437156677246, 1.9976996183395386, 1.9897704124450684,
2.0078816413879395, 2.0032715797424316, 1.9790011644363403, 1.9671071767807007, 1.9633406400680542,
1.9843580722808838, 1.9815183877944946, 1.9709762334823608, 1.9542745351791382, 1.9656703472137451,
1.9690845012664795, 1.9672737121582031, 1.959699273109436, 1.9631857872009277, 1.9696965217590332,
1.976806879043579, 1.9686083793640137, 1.969375491142273, 1.9682399034500122, 1.969983696937561,
1.961828351020813, 1.965994119644165, 1.9675331115722656, 1.985302448272705, 1.9628318548202515,
1.9597870111465454, 1.9727448225021362, 1.9889177083969116, 1.976778507232666, 1.967637300491333,1.9621281623840332,
1.984622836112976, 1.9887748956680298, 1.9951529502868652, 1.9989867210388184, 1.9956954717636108,
1.985696792602539, 2.0129592418670654, 2.0012946128845215, 1.9983521699905396, 2.0063636302948,
2.024601459503174, 1.9984159469604492, 1.9979652166366577, 2.011967420578003, 2.043313980102539,
2.039560317993164, 2.040328025817871, 2.048557758331299, 1.991007924079895, 1.9208488464355469,
1.8859797716140747, 1.8114429712295532, 1.774153470993042, 1.7334085702896118, 1.6852494478225708,
1.640979290008545, 1.594928503036499, 1.5661307573318481, 1.535333275794983, 1.4926395416259766,
1.4703819751739502, 1.4391350746154785, 1.400377631187439, 1.3868000507354736, 1.3687784671783447,
1.326764464378357, 1.3037467002868652, 1.252630352973938, 1.2635324001312256, 1.231882929801941,1.191925048828125,
1.2031205892562866, 1.1917303800582886, 1.1360584497451782, 1.1202435493469238, 1.1178069114685059,
1.1050363779067993, 1.0889618396759033, 1.0617709159851074, 1.0483131408691406, 1.0277975797653198,
1.0492870807647705, 0.9941926002502441, 0.9830083847045898, 0.984307050704956,
0.9492697715759277, 0.9536517262458801, 0.9440097808837891, 0.924079954624176,
0.9140264987945557, 0.9113636612892151, 0.8965656161308289, 0.8869219422340393,
0.8743773102760315, 0.8717730045318604, 0.8754608631134033, 0.8565568327903748,
0.8395403027534485, 0.8398444056510925, 0.839319109916687, 0.8177809715270996,
0.8169471025466919, 0.8096579909324646, 0.7841596603393555, 0.7967863082885742,
0.7749651670455933, 0.7679194808006287, 0.7686342597007751, 0.7667801380157471,
0.7497155666351318, 0.7421659827232361, 0.7340196967124939, 0.7341059446334839,
0.7350730895996094, 0.7253764867782593, 0.7368423938751221, 0.723910391330719,
0.7123515009880066, 0.6871606111526489, 0.6986804008483887, 0.6883254051208496,
0.7066303491592407, 0.6846004724502563, 0.6879106163978577, 0.6708396673202515,
0.6806128621101379, 0.6699912548065186, 0.680689811706543, 0.6723269820213318,
0.6636863946914673, 0.6597433686256409, 0.6448408961296082, 0.657247006893158,
0.6612385511398315, 0.6415248513221741, 0.6293361783027649, 0.6288841962814331,
0.6423088908195496, 0.6149875521659851, 0.6281951069831848, 0.6210680603981018,
0.6183944940567017, 0.6131534576416016, 0.6292304992675781, 0.6150520443916321,
0.6157017946243286, 0.6278789639472961, 0.6088560223579407, 0.5932957530021667,
0.6133307814598083, 0.5877262949943542, 0.5872122049331665, 0.5777624249458313,
0.5838360786437988, 0.5871717929840088, 0.5785636901855469, 0.5734881162643433,
0.5832432508468628, 0.5929278135299683, 0.5673882365226746, 0.5711469054222107,
0.573992908000946, 0.5785197615623474, 0.5744770765304565, 0.5609079599380493,
0.5790647268295288, 0.5594781041145325, 0.5640906691551208, 0.5396769642829895,
0.585415244102478, 0.5798218846321106, 0.5691438913345337, 0.5617644190788269,
0.5623156428337097, 0.570677638053894, 0.5521483421325684, 0.557357907295227,
0.557405948638916, 0.571537435054779, 0.5711507797241211, 0.5489227771759033,
0.5500850677490234, 0.5437018871307373, 0.5422120690345764, 0.549059271812439,
0.5520737767219543, 0.5540491342544556, 0.551235020160675, 0.5599464774131775,
0.5409579873085022, 0.5405648946762085, 0.5496052503585815, 0.5649397373199463,
0.5371201634407043, 0.555336594581604, 0.5549090504646301, 0.5646864175796509,
0.5617849826812744, 0.5438942313194275, 0.5706580281257629, 0.5576022863388062,
0.5650997757911682, 0.5553752779960632, 0.5598331093788147, 0.5489990711212158,
0.5484316945075989, 0.5637081861495972, 0.5433087944984436, 0.5351781249046326,
0.5340425372123718, 0.5621834993362427, 0.5469188094139099, 0.5417068600654602,
0.5456775426864624, 0.5248217582702637, 0.5677840709686279, 0.5583963990211487,
0.5389694571495056, 0.5011442303657532, 0.49343276023864746, 0.454184889793396,
0.4253697395324707, 0.4313235282897949, 0.4231050908565521, 0.3823477625846863,
0.3821916878223419, 0.3683989644050598, 0.3626886308193207, 0.34490418434143066,
0.3232698440551758, 0.3172706663608551, 0.30691081285476685,
0.29870107769966125, 0.32053110003471375, 0.2791930139064789,
0.28141090273857117, 0.25812992453575134, 0.24960507452487946,
0.26512548327445984, 0.25090253353118896, 0.2456735521554947,
0.24617117643356323, 0.24558600783348083, 0.2377818375825882,
0.24737069010734558, 0.2613084316253662, 0.2069551944732666, 0.2133972942829132,
0.20811305940151215, 0.221188023686409, 0.20508308708667755,
0.21498161554336548, 0.19437865912914276, 0.20079763233661652,
0.2022760659456253,0.19250959157943726, 0.17991717159748077, 0.19507996737957,
0.17753049731254578, 0.18278315663337708, 0.17836233973503113,
0.1721208691596985, 0.17265518009662628, 0.16805382072925568,
0.17183269560337067, 0.15343862771987915, 0.17911437153816223,
0.16270792484283447, 0.1647626906633377, 0.14501836895942688,
0.1466057151556015, 0.14852787554264069, 0.14357061684131622,
0.15011680126190186, 0.13182535767555237, 0.15224507451057434,
0.1677912026643753, 0.14233118295669556, 0.1431960016489029,
0.14339838922023773, 0.14728322625160217, 0.15603788197040558,
0.1424553543329239, 0.12939807772636414,0.12501567602157593, 0.1310703009366989,
0.14242610335350037, 0.13483406603336334, 0.12188469618558884,
0.1324913650751114, 0.12808501720428467, 0.13298563659191132,
0.13122674822807312, 0.1395920217037201, 0.12385940551757812,
0.11999999731779099, 0.11999999731779099, 0.11999999731779099,
0.11999999731779099, 0.12230966985225677, 0.11999999731779099,
0.11999999731779099, 0.11999999731779099, 0.1354987472295761,
0.14814722537994385, 0.13587258756160736, 0.1330968141555786,
0.13089509308338165, 0.12326023727655411, 0.12035712599754333,
0.11999999731779099, 0.11999999731779099, 0.11999999731779099,
0.12370899319648743, 0.12654466927051544, 0.12746991217136383,
0.12288372963666916, 0.1307370513677597, 0.1269354373216629,
0.12602627277374268, 0.11999999731779099, inf, inf, inf, inf, inf, inf,inf, inf,
inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,
inf, inf,inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf,inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,inf, inf, inf, inf, inf, inf,
inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,inf, inf,
inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf,
inf, inf,inf, inf, inf, inf, 0.1254916787147522, 0.12653963267803192,
0.12462405860424042, 0.12024720013141632, 0.12901252508163452,
0.13697966933250427, 0.12440931051969528, 0.12616592645645142,
0.13310731947422028, 0.11999999731779099, 0.12087798118591309,
0.13650068640708923, 0.1224832758307457, 0.12320595234632492,
0.12043261528015137, 0.130936399102211, 0.12374472618103027, 0.1235460713505745,
0.12182890623807907, 0.13094507157802582, 0.12136536836624146,
0.1355004459619522, 0.12918302416801453, 0.1404108852148056,
0.12108730524778366, 0.120698481798172, 0.14112672209739685,
0.13148795068264008, 0.13800515234470367, 0.12153178453445435,
0.13117586076259613, 0.12357429414987564, 0.13998600840568542,
0.12877815961837769, 0.1274087280035019, 0.14579415321350098,
0.13656817376613617, 0.14802414178848267, 0.14554989337921143,
0.14951448142528534, 0.13792173564434052, 0.15610451996326447,
0.14787954092025757, 0.15766029059886932, 0.15548324584960938,
0.16415613889694214, 0.15035685896873474, 0.1608540415763855,
0.14933715760707855, 0.14386232197284698, 0.15602794289588928,
0.15185002982616425, 0.1478773057460785, 0.1767287254333496,
0.17662984132766724, 0.16892880201339722, 0.16580693423748016,
0.16586032509803772, 0.17636100947856903, 0.1811331957578659,
0.19079208374023438, 0.1857379972934723, 0.18838262557983398,
0.18178533017635345, 0.20899800956249237, 0.2072417140007019,
0.19307954609394073, 0.21373718976974487, 0.21055322885513306,
0.19948644936084747, 0.20985160768032074, 0.226098895072937,
0.19941231608390808, 0.22132067382335663, 0.2194317728281021,
0.22789306938648224, 0.24046434462070465, 0.2419840395450592,
0.24812708795070648, 0.24274393916130066, 0.24441488087177277,
0.25964266061782837, 0.2535812556743622, 0.29248562455177307,
0.2963385283946991, 0.2891883850097656, 0.29554885625839233,
0.29233378171920776, 0.29013556241989136, 0.31663259863853455,
0.3228529989719391, 0.3177735507488251, 0.366286039352417, 0.36295264959335327,
0.3624456822872162, 0.37869495153427124, 0.41087988018989563,
0.4143538475036621, 0.4166443943977356, 0.43751126527786255, 0.4702695608139038,
0.4656648337841034, 0.4941064417362213, 0.5291009545326233, 0.5383895039558411,
0.5748342871665955, 0.6014317870140076, 0.6390464901924133, 0.6808745265007019,
0.7323399186134338, 0.7790796160697937, 0.8325223326683044, 0.9165522456169128,
0.9990151524543762, 1.064873218536377, 1.2008181810379028, 1.2779085636138916, 1.2638272047042847,
1.2766858339309692, 1.2579210996627808, 1.26985764503479, 1.2741742134094238, 1.2561215162277222,
1.2594664096832275, 1.2704969644546509, 1.2591025829315186, 1.2740148305892944, 1.248815894126892,
1.290112018585205, 1.2740306854248047, 1.264096975326538, 1.2763277292251587, 1.2696930170059204,
1.2696391344070435, 1.2755435705184937, 1.268579363822937, 1.267015814781189, 1.2868494987487793,
1.2765774726867676, 1.2677382230758667, 1.271206021308899, 1.2815710306167603, 1.3015919923782349,
1.2865601778030396, 1.2780989408493042, 1.3049067258834839, 1.3091647624969482, 1.3008601665496826,
1.292917013168335, 1.285374641418457, 1.2910478115081787, 1.2936620712280273, 1.3046954870224,
1.31719970703125, 1.2801697254180908, 1.3068640232086182, 1.3189094066619873, 1.316416621208191,
1.3137309551239014, 1.3371362686157227, 1.3371328115463257, 1.3358479738235474, 1.3414041996002197,
1.3289155960083008, 1.3692947626113892, 1.3459506034851074, 1.3548349142074585, 1.3534013032913208,
1.3796887397766113, 1.3746938705444336, 1.3710174560546875, 1.3874690532684326, 1.3756259679794312,
1.392828345298767, 1.3883973360061646, 1.4055535793304443, 1.4005138874053955, 1.413269281387329,
1.4129273891448975, 1.4268280267715454, 1.4148768186569214, 1.4463456869125366, 1.4439773559570312,
1.4690475463867188, 1.453991413116455, 1.481472134590149, 1.4828617572784424, 1.478192925453186,
1.500072956085205, 1.515499234199524, 1.5137687921524048, 1.5057477951049805, 1.5238914489746094,
1.5358963012695312, 1.5381128787994385, 1.5657938718795776, 1.5534886121749878, 1.5565249919891357,
1.6027525663375854, 1.5884901285171509, 1.6221710443496704, 1.623008131980896, 1.6446263790130615,
1.6339530944824219, 1.6418389081954956, 1.6779470443725586, 1.683975338935852, 1.6809618473052979,
1.7056114673614502, 1.72196626663208, 1.7306268215179443, 1.7371740341186523, 1.7683355808258057,
1.7660468816757202, 1.7774505615234375, 1.792159080505371, 1.844856858253479,1.8483458757400513,
1.8513226509094238, 1.8648912906646729, 1.8859608173370361, 1.9096462726593018,0.7251989245414734,
0.7542835474014282, 0.7193841338157654, 0.7209505438804626,
0.7151182293891907,0.704704999923706, 0.7143740653991699, 0.6941506266593933,
0.6838874816894531, 0.6846728324890137, 0.6905593276023865, 0.665961503982544,
0.6777763962745667,
0.732663094997406, 2.2578277587890625, 2.286787748336792, 2.3181660175323486, 2.334733486175537,
2.3339788913726807, 2.319035291671753, 2.2687065601348877, 2.290625810623169, 2.264962911605835,
2.2606780529022217, 2.2532873153686523, 2.240159749984741, 2.2214505672454834, 2.206315755844116,
2.195305824279785, 2.1984171867370605, 2.1882081031799316, 2.1766369342803955, 2.158062696456909,
2.152400255203247, 2.1502580642700195, 2.15509295463562, 0.603262722492218,
0.5971719026565552, 0.6061855554580688, 0.5799170136451721, 0.6060362458229065,
0.5993317365646362, 0.5953881740570068, 0.5759713053703308, 0.5961670875549316,
0.5816265940666199, 0.5952827334403992, 0.5935145616531372, 0.5668087005615234,
0.5770943760871887, 0.5594232678413391, 0.5743329524993896, 0.5934719443321228,
0.5827717185020447, 0.5722882747650146, 1.0728917121887207] intensities: [0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
*/