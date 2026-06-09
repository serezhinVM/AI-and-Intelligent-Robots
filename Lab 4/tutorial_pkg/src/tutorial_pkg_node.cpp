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
std::ofstream robotLaserFile; // Declare a file object for recording your laser data.
std::ofstream laserMapFile;

struct EulerAngles{double roll, pitch, yaw;};  // yaw is what you want, i.e. Th  
struct Quaternion{double w, x, y, z;};  

EulerAngles ToEulerAngles(Quaternion q){  // for calculating Th 
    EulerAngles angles; 
    // roll (x-axis rotation) 
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z); 
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y); 
    angles.roll = atan2(sinr_cosp, cosr_cosp); 
    // pitch (y-axis rotation) 
    double sinp = +2.0 * (q.w * q.y - q.z * q.x); 
    if (fabs(sinp) >= 1) 
        angles.pitch = copysign(M_PI/2, sinp);
    else 
        angles.pitch = asin(sinp); 
    // yaw (z-axis rotation) 
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y); 
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);   
    angles.yaw = atan2(siny_cosp, cosy_cosp); 
    return angles; 
} 


using namespace std::chrono_literals;
using namespace std;

class Stopper : public rclcpp::Node{
public:
    constexpr const static double FORWARD_SPEED_LOW = 0.1;
    constexpr const static double FORWARD_SPEED_MIDDLE = 0.3;
    constexpr const static double FORWARD_SPEED_HIGH = 0.5;
    constexpr const static double FORWARD_SPEED_STOP = 0;
    constexpr const static double TURN_LEFT_SPEED_LOW = 0.3;
    constexpr const static double TURN_LEFT_SPEED_MIDDLE = 0.6;
    constexpr const static double TURN_LEFT_SPEED_HIGH = 1.0;
    constexpr const static double TURN_RIGHT_SPEED_LOW = -0.3;
    constexpr const static double TURN_RIGHT_SPEED_MIDDLE = -0.6;
    constexpr const static double TURN_RIGHT_SPEED_HIGH = -1.0;

    Stopper():Node("Stopper"), count_(0){
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, bind(&Stopper::odomCallback, this, placeholders::_1));
        laserScan_=this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Stopper::scanCallback, this, std::placeholders::_1));
    };

    void startMoving();
    void moveStop();
    void moveForward(double forwardSpeed);
    void moveRight(double turn_right_speed);
    void moveForwardRight(double forwardSpeed, double turn_right_speed);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
    double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
    double odom_landmark1=1.38, odom_landmark2=0.80;
    double odom_landmark3=1.3, odom_landmark4=1.72, odom_landmark5=0.3;
    int stage=1;
    double robVelocity;
    double frontRange, mleftRange, leftRange, rightRange, mrightRange; 
    int laser_index = 0; // index the laser scan data
    /* moving function */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void transformMapPoint(ofstream& fp, double laserRange, double laserTh, double robotTh, double robotX, double robotY);
    
    Quaternion robotQuat; 
    EulerAngles robotAngles;  
    double robotHeadAngle; 
    double leftAngle=M_PI/2, mleftAngle=M_PI/4, frontAngle=0; 
    double mrightAngle=-M_PI/4, rightAngle=-M_PI/2; 
    
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

void Stopper::moveForward(double forwardSpeed){
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    publisher_->publish(msg);
}

void Stopper::moveRight(double turn_right_speed){
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = turn_right_speed;
    publisher_->publish(msg);
}

void Stopper::moveForwardRight(double forwardSpeed, double turn_right_speed){
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forwardSpeed;
    msg.angular.z = turn_right_speed;
    publisher_->publish(msg);
}

void Stopper::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg){
    PositionX = odomMsg->pose.pose.position.x + homeX;
    PositionY = odomMsg->pose.pose.position.y + homeY;
    RCLCPP_INFO(this->get_logger(), "RobotPosition: %.2f , %.2f", PositionX, PositionY);
    RCLCPP_INFO(this->get_logger(), "Robot stage: %d ", stage);
    robotTrajFile << PositionX <<" "<<PositionY<<endl;
    robotVelFile << ++count_ <<" "<< odomMsg->twist.twist.linear.x <<" "<< odomMsg->twist.twist.angular.z <<endl;
    robotQuat.x = odomMsg->pose.pose.orientation.x; 
    robotQuat.y = odomMsg->pose.pose.orientation.y; 
    robotQuat.z = odomMsg->pose.pose.orientation.z; 
    robotQuat.w = odomMsg->pose.pose.orientation.w; 
    robotAngles = ToEulerAngles(robotQuat); 
    robotHeadAngle = robotAngles.yaw; 
    
    switch(stage){
        case 1:
            if (PositionY < odom_landmark1)
                moveForward(FORWARD_SPEED_MIDDLE);
            else stage = 2;
            break;
        case 2:
            if (PositionX < odom_landmark2)
                moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
            else stage = 3;
            break;
        case 3:
            if (PositionX < odom_landmark3)
                moveForward(FORWARD_SPEED_HIGH);
            else stage = 4;
            break;
        case 4:
            if (PositionX < odom_landmark4)
                moveForwardRight(FORWARD_SPEED_MIDDLE, TURN_RIGHT_SPEED_MIDDLE);
            else stage = 5;
            break;
        case 5:
            if (PositionY > odom_landmark5)
                moveForward(FORWARD_SPEED_LOW);
            else stage = 6;
            break;
        case 6:
            moveStop();
            break;
    }
}

void Stopper::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){ 
	leftRange = scan->ranges[90]; // get a range reading at the left angle 
	mleftRange = 	scan->ranges[45]; // get a range reading at the mid-left angle
	frontRange = scan->ranges[0]; // get a range reading at the front angle 
	mrightRange = scan->ranges[315]; // get a range reading at the mid-right angle
	rightRange = scan->ranges[270]; // get the range reading at the right angle
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
    RCLCPP_INFO(this->get_logger(), "Start moving");
    robotTrajFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotTrajData.csv", ios::trunc);
    robotVelFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotVelData.csv", ios::trunc);
    robotLaserFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laserData.csv",ios::trunc);
    laserMapFile.open("/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laserMapData.csv",ios::trunc);
    while (rclcpp::ok()){   // keep spinning loop until user presses Ctrl+C
        rclcpp::WallRate loop_rate(20);
        auto node = std::make_shared<Stopper>();
        rclcpp::spin(node);
        loop_rate.sleep();
    }
    robotTrajFile.close();
    robotVelFile.close();
    robotLaserFile.close();
    laserMapFile.close();
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    Stopper stopper;
    stopper.startMoving();
    return 0;
}
