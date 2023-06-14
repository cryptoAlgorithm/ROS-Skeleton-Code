#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#define NODE_NAME "robot_controller_node"

#define ANGULAR_RAD      10
#define ANGULAR_FACTOR 1.0f
#define LINEAR_FACTOR  1.0f
#define MAX_DUTY_CYCLE 1.0f
#define MIN_DUTY_CYCLE 0.6f

#define MIN(a, b) (a < b ? a : b)
#define MAX(a, b) (a > b ? a : b)
#define CLAMP(x, min, max) (MIN(max, MAX(min, x)))

ros::Publisher duty_cycle_pub;

void calculatePWM(float lin_vel, float ang_vel)
{
    // Because its not possible to move with the accuracy of any unit, we try to at
    // least ensure linear and angular velocity is approximately proportional
    // +ve duty cycle: forward; -ve duty cycle: backward
    float left_wheel_duty_cycle = lin_vel*LINEAR_FACTOR - ang_vel*ANGULAR_RAD*ANGULAR_FACTOR,
          right_wheel_duty_cycle = lin_vel*LINEAR_FACTOR + ang_vel*ANGULAR_RAD*ANGULAR_FACTOR;
 
    // Construct pub msg according to format "left_wheel_duty_cycle,right_wheel_duty_cycle"
    std_msgs::String duty_cycle_command;
    duty_cycle_command.data =
        std::to_string(CLAMP(left_wheel_duty_cycle, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE))
        + ","
        + std::to_string(CLAMP(right_wheel_duty_cycle, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE));
    duty_cycle_pub.publish(duty_cycle_msg);
    ROS_INFO("Published duty cycle (L,R): %s", duty_cycle_command.data.c_str());
}

void subCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // only the following 2 fields of the msg are used 
    // the rest are all 0
    // "Ok good to know"
    float linear_vel = msg->linear.x;
    float angular_vel = msg->angular.z;

    ROS_INFO("Received velocity command: [lin_vel = %f] [ang_vel = %f]", linear_vel, angular_vel);
    // ROS_INFO("Received keyboard command!");

    calculatePWM(linear_vel, angular_vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ROS_INFO("Started "NODE_NAME"!");

    ros::NodeHandle n;

    // Init duty cycle pub
    duty_cycle_pub = n.advertise<std_msgs::String>("duty_cycle", 100); // We probably don't need that big a buff
    // Sub to keyboard callbacks
    ros::Subscriber sub = n.subscribe("cmd_vel", 100, subCallback);

    ros::spin(); // Loop forever while fulfilling callbacks

    return 0;
}
