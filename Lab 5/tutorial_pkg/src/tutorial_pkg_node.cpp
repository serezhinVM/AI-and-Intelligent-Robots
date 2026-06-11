#include <chrono> 
#include <functional> 
#include <memory> 
#include <string> 
#include <fstream> 
#include <time.h> 
#include <iomanip> 
#include <cmath>
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp" 
#include "geometry_msgs/msg/twist.hpp" 
#include "geometry_msgs/msg/pose.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

std::ofstream robotTrajFile;
std::ofstream robotVelFile;
std::ofstream robotLaserFile;
std::ofstream laserMapFile;
std::ofstream robotHeadFile;

struct EulerAngles{double roll, pitch, yaw;};
struct Quaternion{double w, x, y, z;};

EulerAngles ToEulerAngles(Quaternion q){
    EulerAngles angles;
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI/2, sinp);
    else
        angles.pitch = asin(sinp);
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    return angles;
}

struct PID_para{double kp, ki, kd, ei_pre, ed_pre, Max_output;};

double PID_control(PID_para pid, double setValue, double measuredData){
    double err = setValue - measuredData;
    double ei = pid.ei_pre + err;
    double ed = err - pid.ed_pre;
    double output = pid.kp*err + pid.ki*ei + pid.kd*ed;
    if (output > pid.Max_output)
        output = pid.Max_output;
    else if(output < -pid.Max_output)
        output = -pid.Max_output;
    pid.ei_pre = ei;
    pid.ed_pre = ed;
    return output;
}

using namespace std::chrono_literals;
using namespace std;

class Stopper : public rclcpp::Node{
public:
    constexpr const static double FORWARD_SPEED_LOW = 0.1;
    constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
    constexpr const static double FORWARD_SPEED_HIGH = 0.5;
    constexpr const static double FORWARD_SPEED_STOP = 0;

    Stopper():Node("Stopper"), count_(0){
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, bind(&Stopper::odomCallback, this, placeholders::_1));
        laserScan_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
    };

    void startMoving();
    void moveStop();
    void moveForwardRight(double forwardSpeed, double turn_speed);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);
    void PID_wallFollowing(double forwardSpeed, double laserData);
    void PID_heading(PID_para pid, double moveSpeed, double targetHeading);

    double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
    double odom_landmark1=1.38, odom_landmark2=0.95;
    double odom_landmark3=1.55, odom_landmark4=1.95, odom_landmark5=0.33;
    int stage=1;
    double robVelocity;
    double frontRange, mleftRange, leftRange, rightRange, mrightRange;
    int laser_index = 0;

    Quaternion robotQuat;
    EulerAngles robotAngles;
    double robotHeadAngle;
    double leftAngle=M_PI/2, mleftAngle=M_PI/4, frontAngle=0;
    double mrightAngle=-M_PI/4, rightAngle=-M_PI/2;
    double heading_gap1 = 0.0;
    double heading_gap2 = -0.80;
    double heading_charger = -1.45;
    double wall_target = 0.30;

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    size_t count_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_;
};

void Stopper::moveStop(){
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = FORWARD_SPEED_STOP;
    publisher_->publish(msg);
}

void Stopper::moveForwardRight(double forwardSpeed, double turn_speed){
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    msg.angular.z = turn_speed;
    publisher_->publish(msg);
}

void Stopper::PID_wallFollowing(double forwardSpeed, double laserData){
    PID_para c{0.5, 0.01, 0.01, 0, 0, 0.6};
    double out = PID_control(c, wall_target, laserData);
    moveForwardRight(forwardSpeed, -out);
}

void Stopper::PID_heading(PID_para pid, double moveSpeed, double targetHeading){
    double out = PID_control(pid, targetHeading, robotHeadAngle);
    moveForwardRight(moveSpeed, out);
}

void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg){
    PositionX = odomMsg->pose.pose.position.x + homeX;
    PositionY = odomMsg->pose.pose.position.y + homeY;
    RCLCPP_INFO(this->get_logger(), "RobotPosition: %.2f , %.2f", PositionX, PositionY);
    RCLCPP_INFO(this->get_logger(), "Robot stage: %d ", stage);
    robotTrajFile << PositionX <<" "<<PositionY<<endl;
    robVelocity = odomMsg->twist.twist.linear.x;
    robotVelFile << ++count_ <<" "<< robVelocity <<" "<< odomMsg->twist.twist.angular.z <<endl;
    robotQuat.x = odomMsg->pose.pose.orientation.x;
    robotQuat.y = odomMsg->pose.pose.orientation.y;
    robotQuat.z = odomMsg->pose.pose.orientation.z;
    robotQuat.w = odomMsg->pose.pose.orientation.w;
    robotAngles = ToEulerAngles(robotQuat);
    robotHeadAngle = robotAngles.yaw;
    robotHeadFile << count_ <<" "<< robotHeadAngle << endl;

    PID_para hpid{2.0, 0.05, 0.01, 0, 0, 0.6};
    switch(stage){
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

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    leftRange = scan->ranges[90];
    mleftRange = scan->ranges[45];
    frontRange = scan->ranges[0];
    mrightRange = scan->ranges[315];
    rightRange = scan->ranges[270];
    robotLaserFile << leftRange << ","<< mleftRange << "," << frontRange<<"," << mrightRange << "," << rightRange <<"," <<laser_index++<< endl;
    transformMapPoint(laserMapFile,frontRange,frontAngle,robotHeadAngle,PositionX, PositionY);
    transformMapPoint(laserMapFile, mleftRange, mleftAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, leftRange, leftAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, rightRange, rightAngle, robotHeadAngle, PositionX, PositionY);
    transformMapPoint(laserMapFile, mrightRange, mrightAngle, robotHeadAngle, PositionX, PositionY);
}

void Stopper::transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY){
    double transX, transY;
    transX = laserRange * cos(robotTh + laserTh) + robotX;
    transY = laserRange * sin(robotTh + laserTh) + robotY;
    if (transX < 0) transX = 0;
    if (transY < 0) transY = 0;
    fp << transX << ", " << transY << endl;
}

void Stopper::startMoving(){
    RCLCPP_INFO(this->get_logger(), "Start moving (PID control)");
    robotTrajFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotTrajData.csv", ios::trunc);
    robotVelFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotVelData.csv", ios::trunc);
    robotLaserFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laserData.csv",ios::trunc);
    laserMapFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laserMapData.csv",ios::trunc);
    robotHeadFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotHeadData.csv",ios::trunc);
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Stopper>();
    node->startMoving();
    rclcpp::spin(node);
    robotTrajFile.close();
    robotVelFile.close();
    robotLaserFile.close();
    laserMapFile.close();
    robotHeadFile.close();
    rclcpp::shutdown();
    return 0;
}
