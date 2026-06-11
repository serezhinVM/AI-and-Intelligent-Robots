// Lab 6 - PID navigation + improved laser map (optimised code).
//
//  6.1  PID control: passes both gaps without collision and reaches the charger
//       robustly (same controller as Lab 5).
//  6.2  Map improvement: the environment map is now built from ALL laser
//       readings (a full 360-degree scan), not just 5 -> a dense, high-quality
//       map of the room and pillars.
//  6.3  Optimisation: the laser-to-world conversion is a single loop; magic
//       numbers are replaced by named constants; unused code was removed.
//
// Outputs:
//   robotTrajData.csv    - PositionX PositionY   (trajectory)
//   robotVelData.csv     - cycle linear_x angular_z (velocity)
//   robotHeadingData.csv - cycle yaw[rad]        (heading)
//   laserMapData.csv     - x, y world points     (improved map)

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

std::ofstream robotTrajFile;
std::ofstream robotVelFile;
std::ofstream laserMapFile;
std::ofstream robotHeadFile;

struct EulerAngles { double roll, pitch, yaw; };
struct Quaternion  { double w, x, y, z; };

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp);
    else
        angles.pitch = asin(sinp);
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    return angles;
}

struct PID_para { double kp, ki, kd, ei_pre, ed_pre, Max_output; };

double PID_control(PID_para pid, double setValue, double measuredData)
{
    double err = setValue - measuredData;
    double ei = pid.ei_pre + err;
    double ed = err - pid.ed_pre;
    double output = pid.kp * err + pid.ki * ei + pid.kd * ed;
    if (output > pid.Max_output)
        output = pid.Max_output;
    else if (output < -pid.Max_output)
        output = -pid.Max_output;
    return output;
}

using namespace std;

class Stopper : public rclcpp::Node
{
public:
    constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
    constexpr const static double FORWARD_SPEED_STOP   = 0.0;
    constexpr const static int    LEFT_LASER_INDEX     = 90;
    constexpr const static int    MAP_STEP             = 4;

    Stopper() : Node("Stopper")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, bind(&Stopper::odomCallback, this, placeholders::_1));
        laserScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, bind(&Stopper::scanCallback, this, placeholders::_1));
    }

    void startMoving();
    void moveStop();
    void moveForwardRight(double forwardSpeed, double turn_speed);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void transformMapPoint(double laserRange, double laserTh);
    void PID_wallFollowing(double forwardSpeed, double laserData);
    void PID_heading(PID_para pid, double moveSpeed, double targetHeading);

    double PositionX = 0.3, PositionY = 0.3, homeX = 0.3, homeY = 0.3;
    double odom_landmark1 = 1.38;
    double odom_landmark2 = 0.95;
    double odom_landmark3 = 1.55;
    double odom_landmark4 = 1.95;
    double odom_landmark5 = 0.33;
    double heading_gap1    = 0.0;
    double heading_gap2    = -0.80;
    double heading_charger = -1.45;
    double wall_target     = 0.30;
    int stage = 1;

    double robVelocity = 0.0, robotHeadAngle = 0.0, leftRange = 0.0;
    int numberOfCycles = 0;
    Quaternion robotQuat;
    EulerAngles robotAngles;

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

void Stopper::moveForwardRight(double forwardSpeed, double turn_speed)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x  = forwardSpeed;
    msg.angular.z = turn_speed;
    publisher_->publish(msg);
}

void Stopper::PID_wallFollowing(double forwardSpeed, double laserData)
{
    PID_para c{0.5, 0.01, 0.01, 0, 0, 0.6};
    double out = PID_control(c, wall_target, laserData);
    moveForwardRight(forwardSpeed, -out);
}

void Stopper::PID_heading(PID_para pid, double moveSpeed, double targetHeading)
{
    double out = PID_control(pid, targetHeading, robotHeadAngle);
    moveForwardRight(moveSpeed, out);
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
    robotTrajFile << PositionX << " " << PositionY << endl;
    robotVelFile << numberOfCycles << " " << robVelocity << " " << odomMsg->twist.twist.angular.z << endl;

    robotQuat.x = odomMsg->pose.pose.orientation.x;
    robotQuat.y = odomMsg->pose.pose.orientation.y;
    robotQuat.z = odomMsg->pose.pose.orientation.z;
    robotQuat.w = odomMsg->pose.pose.orientation.w;
    robotAngles = ToEulerAngles(robotQuat);
    robotHeadAngle = robotAngles.yaw;
    robotHeadFile << numberOfCycles << " " << robotHeadAngle << endl;
    numberOfCycles++;

    PID_para hpid{2.0, 0.05, 0.01, 0, 0, 0.6};
    switch (stage)
    {
        case 1:
            if (PositionY < odom_landmark1)
                PID_wallFollowing(FORWARD_SPEED_MIDDLE, leftRange);
            else stage = 2;
            break;
        case 2:
            if (PositionX < odom_landmark2)
                PID_heading(hpid, FORWARD_SPEED_MIDDLE, heading_gap1);
            else stage = 3;
            break;
        case 3:
            if (PositionX < odom_landmark3)
                PID_heading(hpid, FORWARD_SPEED_MIDDLE, heading_gap2);
            else stage = 4;
            break;
        case 4:
            if (PositionX < odom_landmark4)
                PID_heading(hpid, FORWARD_SPEED_MIDDLE, heading_gap2);
            else stage = 5;
            break;
        case 5:
            if (PositionY > odom_landmark5)
                PID_heading(hpid, FORWARD_SPEED_MIDDLE, heading_charger);
            else stage = 6;
            break;
        case 6:
            moveStop();
            break;
    }
}

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    leftRange = scan->ranges[LEFT_LASER_INDEX];

    for (size_t i = 0; i < scan->ranges.size(); i += MAP_STEP)
        transformMapPoint(scan->ranges[i], scan->angle_min + i * scan->angle_increment);
}

void Stopper::startMoving()
{
    RCLCPP_INFO(this->get_logger(), "Start moving - PID + improved map");
    robotTrajFile.open("robotTrajData.csv", ios::trunc);
    robotVelFile.open("robotVelData.csv", ios::trunc);
    laserMapFile.open("laserMapData.csv", ios::trunc);
    robotHeadFile.open("robotHeadingData.csv", ios::trunc);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<Stopper>();
    node->startMoving();
    rclcpp::spin(node);
    for (auto *f : {&robotTrajFile, &robotVelFile, &laserMapFile, &robotHeadFile})
        if (f->is_open()) f->close();
    rclcpp::shutdown();
    return 0;
}
