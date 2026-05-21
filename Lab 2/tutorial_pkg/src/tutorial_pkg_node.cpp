#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    };

    void startMoving();
    void moveStop();
    void moveForward(double forwardSpeed);
    void moveRight(double turn_right_speed);
    void moveForwardRight(double forwardSpeed, double turn_right_speed);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg);

    double PositionX=0.3, PositionY=0.3, homeX=0.3, homeY=0.3;
    double odom_landmark1=1.38, odom_landmark2=0.80;
    double odom_landmark3=0.9, odom_landmark4=1.3, odom_landmark5=0.3;
    int stage=1;

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
    size_t count_;
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

void Stopper::startMoving(){
    RCLCPP_INFO(this->get_logger(), "Start moving");
    while (rclcpp::ok()){
        rclcpp::WallRate loop_rate(20);
        auto node = std::make_shared<Stopper>();
        rclcpp::spin(node);
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    Stopper stopper;
    stopper.startMoving();
    return 0;
}
