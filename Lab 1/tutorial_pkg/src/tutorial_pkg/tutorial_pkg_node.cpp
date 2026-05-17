#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
using namespace std::chrono_literals;
class Stopper : public rclcpp::Node{
public:
    /* velocity control variables*/
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
    /* class constructor */
    Stopper():Node("Stopper"), count_(0){
    publisher_ =this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);};
    /* moving function */
    void startMoving();
    void moveStop();
    void moveForward(double forwardSpeed);
    void moveRight(double turn_right_speed);

private:
    // Publisher to the robot's velocity command topic
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    };

void Stopper::moveStop(){
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = FORWARD_SPEED_STOP;
    publisher_->publish(msg);
}

void Stopper::moveForward(double forwardSpeed){
    //The default constructor to set all commands to 0
    auto msg=geometry_msgs::msg::Twist();
    //Drive forward at a given speed along the x-axis.
    msg.linear.x = forwardSpeed;
    publisher_->publish(msg);
}

void Stopper::moveRight(double turn_right_speed){
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = turn_right_speed;
    publisher_->publish(msg);
}

void Stopper::startMoving(){
    RCLCPP_INFO(this->get_logger(), "Start moving");
    while (rclcpp::ok()){
    rclcpp::WallRate loop_rate(20); // Create a loop running at 20Hz
    moveForward(FORWARD_SPEED_LOW);
    RCLCPP_INFO(this->get_logger(), "Robot speed: %f", FORWARD_SPEED_LOW);
    auto node = std::make_shared<Stopper>(); // Create a new Stopper node
    rclcpp::spin_some(node); // Spins the node to process incoming ROS messages.
    loop_rate.sleep(); // Sleeps for the remaining time in the 20Hz cycle
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv); // Initialize the ROS2 system
    Stopper stopper; // Create an instance of Stopper
    stopper.startMoving(); // Begins the main loop
    return 0; // Indicates successful program termination
}
