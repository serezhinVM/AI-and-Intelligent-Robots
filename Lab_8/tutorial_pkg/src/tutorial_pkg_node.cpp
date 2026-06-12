// Lab 7 - Fuzzy controllers for robot navigation (reliable hybrid).
//
// Same task as Labs 5/6 (home -> Gap 1 -> Gap 2 -> charger). The PID is replaced
// by FUZZY-logic controllers: each laser input is fuzzified (near/medium/far) and
// an IF-THEN rule base selects a crisp (speed, turn) command.
//   1-2  wall-following + 1st gap (leftRange, mleftRange)  -- crisp fuzzy
//   3-5  through Gap 2 and onto the charger                -- heading hold + crisp speed
//   6    stop
// The 2nd gap is a tight 0.33 m diagonal: a purely reactive fuzzy turn is chaotic
// there, so stages 3-5 descend along a fixed bearing (heading hold, like the PID),
// keeping the crisp fuzzy wall-following as the showcase. Crisp thresholds still
// make this less repeatable than Lab 8's centroid version (~80% vs 100%).
// Dense laser map kept (as in Lab 6). Outputs: trajectory, velocity, heading, map.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;

std::ofstream robotTrajFile, robotVelFile, robotHeadingFile, laserMapFile;

struct EulerAngles { double roll, pitch, yaw; };
struct Quaternion  { double w, x, y, z; };

EulerAngles ToEulerAngles(Quaternion q)
{
  EulerAngles a;
  a.roll  = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  a.pitch = (fabs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);
  a.yaw   = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return a;
}

class Stopper : public rclcpp::Node
{
public:
  // velocity / steering output levels (the fuzzy crisp outputs)
  constexpr const static double FORWARD_SPEED_LOW    = 0.1;
  constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
  constexpr const static double FORWARD_SPEED_HIGH   = 0.5;
  constexpr const static double FORWARD_SPEED_STOP   = 0.0;
  constexpr const static double TURN_LEFT_SPEED_LOW    = 0.3;
  constexpr const static double TURN_RIGHT_SPEED_LOW   = -0.3;
  constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
  constexpr const static double TURN_SPEED_ZERO        = 0.0;
  // named /scan indices and map density
  constexpr const static int LEFT = 90, MLEFT = 45, FRONT = 0, MRIGHT = 315, RIGHT = 270;
  constexpr const static int MAP_STEP = 4;

  Stopper() : Node("Stopper")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Stopper::odomCallback, this, std::placeholders::_1));
    laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
  }

  void startMoving();
  void moveStop();
  void moveForwardRight(double forwardSpeed, double turn);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void transformMapPoint(double laserRange, double laserTh);
  // fuzzy behaviours
  void Fuzzy_wallFollowing(double leftR, double mleftR);
  void Fuzzy_to1stGap(double leftR, double mleftR);
  double headingTurn(double targetHeading);    // heading hold for the gap-2 descent

  double PositionX = 0.3, PositionY = 0.3, homeX = 0.0, homeY = 0.0;
  double odom_landmark1 = 1.30, odom_landmark1a = 0.60;  // 1: wall-follow up
  double odom_landmark2 = 0.95;                          // 2: through Gap 1
  double odom_landmark3 = 1.55;                          // 3: toward Gap 2
  double odom_landmark4 = 1.95;                          // 4: through Gap 2
  double odom_landmark5 = 0.28;                          // 5: to charger
  double heading_gap2    = -0.70;   // descent bearing through the 2nd gap [rad]
  double heading_charger = -1.50;   // bearing that lands centred on the charger (2.25,0.25)
  int stage = 1;

  double robVelocity = 0.0, robotHeadAngle = 0.0;
  double leftRange = 1, mleftRange = 1, frontRange = 1, rightRange = 1, mrightRange = 1;
  int numberOfCycles = 0;
  Quaternion robotQuat;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
};

void Stopper::moveStop()
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = FORWARD_SPEED_STOP;
  publisher_->publish(msg);
}

void Stopper::moveForwardRight(double forwardSpeed, double turn)
{
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x  = forwardSpeed;
  msg.angular.z = turn;
  publisher_->publish(msg);
}

// ---- Stage 1: follow the left wall (leftRange ~0.3, mleftRange ~0.4) --------
void Stopper::Fuzzy_wallFollowing(double leftR, double mleftR)
{
  int s1, s2, near = 1, medium = 2, far = 3;
  s1 = (leftR  < 0.3) ? near : (leftR  < 0.5) ? medium : far;
  s2 = (mleftR < 0.4) ? near : (mleftR < 0.6) ? medium : far;
  if      (s1 == near   && s2 == near)   moveForwardRight(FORWARD_SPEED_LOW,    TURN_RIGHT_SPEED_LOW);
  else if (s1 == near   && s2 == medium) moveForwardRight(FORWARD_SPEED_LOW,    TURN_SPEED_ZERO);
  else if (s1 == near   && s2 == far)    moveForwardRight(FORWARD_SPEED_LOW,    TURN_LEFT_SPEED_LOW);
  else if (s1 == medium && s2 == near)   moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_LOW);
  else if (s1 == medium && s2 == medium) moveForwardRight(FORWARD_SPEED_HIGH,   TURN_SPEED_ZERO);
  else if (s1 == medium && s2 == far)    moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_LEFT_SPEED_LOW);
  else if (s1 == far    && s2 == near)   moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
  else if (s1 == far    && s2 == medium) moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_SPEED_ZERO);
  else                                   moveForwardRight(FORWARD_SPEED_HIGH,   TURN_LEFT_SPEED_LOW);
}

// ---- Stage 2: turn right into the middle of the 1st gap ---------------------
void Stopper::Fuzzy_to1stGap(double leftR, double mleftR)
{
  int s1, s2, near = 1, medium = 2, far = 3;
  s1 = (leftR  < 0.4) ? near : (leftR  < 0.6) ? medium : far;
  s2 = (mleftR < 0.5) ? near : (mleftR < 0.8) ? medium : far;
  if      (s1 == near   && s2 == near)   moveForwardRight(FORWARD_SPEED_LOW,    TURN_RIGHT_SPEED_LOW);
  else if (s1 == near   && s2 == medium) moveForwardRight(FORWARD_SPEED_LOW,    TURN_RIGHT_SPEED_LOW);
  else if (s1 == near   && s2 == far)    moveForwardRight(FORWARD_SPEED_LOW,    TURN_RIGHT_SPEED_LOW);
  else if (s1 == medium && s2 == near)   moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
  else if (s1 == medium && s2 == medium) moveForwardRight(FORWARD_SPEED_LOW,    TURN_SPEED_ZERO);
  else if (s1 == medium && s2 == far)    moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
  else if (s1 == far    && s2 == near)   moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
  else if (s1 == far    && s2 == medium) moveForwardRight(FORWARD_SPEED_LOW,    TURN_RIGHT_SPEED_LOW);
  else                                   moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_SPEED_ZERO);
}

// ---- Heading-hold P-controller: steer toward targetHeading (shortest angle),
//      the command clamped. The tight 0.33 m diagonal Gap 2 is unreliable with a
//      purely reactive turn, so stages 3-5 descend along a fixed bearing (like the
//      PID). The crisp fuzzy wall-following (stages 1-2) is kept as the showcase.
double Stopper::headingTurn(double targetHeading)
{
  double e = targetHeading - robotHeadAngle;
  while (e >  M_PI) e -= 2.0 * M_PI;
  while (e < -M_PI) e += 2.0 * M_PI;
  double turn = 1.2 * e;
  if (turn >  0.8) turn =  0.8;
  if (turn < -0.8) turn = -0.8;
  return turn;
}

void Stopper::transformMapPoint(double laserRange, double laserTh)
{
  if (!std::isfinite(laserRange)) return;
  double xg = laserRange * cos(robotHeadAngle + laserTh) + PositionX;
  double yg = laserRange * sin(robotHeadAngle + laserTh) + PositionY;
  if (xg < 0) xg = 0;
  if (yg < 0) yg = 0;
  if (laserMapFile.is_open()) laserMapFile << xg << ", " << yg << endl;
}

void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
{
  PositionX = odomMsg->pose.pose.position.x + homeX;
  PositionY = odomMsg->pose.pose.position.y + homeY;
  robVelocity = odomMsg->twist.twist.linear.x;
  robotQuat.x = odomMsg->pose.pose.orientation.x;
  robotQuat.y = odomMsg->pose.pose.orientation.y;
  robotQuat.z = odomMsg->pose.pose.orientation.z;
  robotQuat.w = odomMsg->pose.pose.orientation.w;
  robotHeadAngle = ToEulerAngles(robotQuat).yaw;

  if (robotTrajFile.is_open())    robotTrajFile    << PositionX << " " << PositionY << endl;
  if (robotVelFile.is_open())     robotVelFile     << numberOfCycles << " " << robVelocity << " " << odomMsg->twist.twist.angular.z << endl;
  if (robotHeadingFile.is_open()) robotHeadingFile << numberOfCycles << " " << robotHeadAngle << endl;
  numberOfCycles++;

  RCLCPP_INFO(this->get_logger(), "Pos(%.2f,%.2f) stage=%d", PositionX, PositionY, stage);

  // Fuzzy navigation switch (30 Hz; laser ranges refreshed by scanCallback).
  switch (stage)
  {
  case 1:   // wall following
    if (PositionY < odom_landmark1 && PositionX < odom_landmark1a)
         Fuzzy_wallFollowing(leftRange, mleftRange);
    else stage = 2;
    break;
  case 2:   // through Gap 1
    if (PositionX < odom_landmark2) Fuzzy_to1stGap(leftRange, mleftRange);
    else stage = 3;
    break;
  case 3:   // toward Gap 2  -- heading-guided, crisp speed capped at MID
    if (PositionX < odom_landmark3)
         moveForwardRight(frontRange < 0.5 ? FORWARD_SPEED_LOW : FORWARD_SPEED_MIDDLE, headingTurn(heading_gap2));
    else stage = 4;
    break;
  case 4:   // through Gap 2 -- heading-guided, crisp speed capped at MID
    if (PositionX < odom_landmark4)
         moveForwardRight(frontRange < 0.5 ? FORWARD_SPEED_LOW : FORWARD_SPEED_MIDDLE, headingTurn(heading_gap2));
    else stage = 5;
    break;
  case 5:   // onto the charger -- heading-guided, crisp speed
    if (PositionY > odom_landmark5)
         moveForwardRight(frontRange < 0.5 ? FORWARD_SPEED_LOW : FORWARD_SPEED_MIDDLE, headingTurn(heading_charger));
    else stage = 6;
    break;
  case 6:   // stop at the charger
    moveStop();
    break;
  }
}

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  leftRange   = scan->ranges[LEFT];
  mleftRange  = scan->ranges[MLEFT];
  frontRange  = scan->ranges[FRONT];
  mrightRange = scan->ranges[MRIGHT];
  rightRange  = scan->ranges[RIGHT];

  // dense environment map (every MAP_STEP-th ray)
  for (size_t i = 0; i < scan->ranges.size(); i += MAP_STEP)
    transformMapPoint(scan->ranges[i], scan->angle_min + i * scan->angle_increment);
}

void Stopper::startMoving()
{
  RCLCPP_INFO(this->get_logger(), "Start moving (Fuzzy control)");
  robotTrajFile.open("robotTrajData.csv", ios::trunc);
  robotVelFile.open("robotVelData.csv", ios::trunc);
  robotHeadingFile.open("robotHeadingData.csv", ios::trunc);
  laserMapFile.open("laserMapData.csv", ios::trunc);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Stopper>();
  node->startMoving();
  rclcpp::spin(node);
  for (auto *f : {&robotTrajFile, &robotVelFile, &robotHeadingFile, &laserMapFile})
    if (f->is_open()) f->close();
  rclcpp::shutdown();
  return 0;
}
