#ifndef GC2_TURTLEBOT_DEMO
#define GC2_TURTLEBOT_DEMO

#include <iostream>

#include <ros/ros.h>
#include <termios.h>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>

#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

namespace gc2_demo {

class TurtlebotDemo {
public:
    TurtlebotDemo();
    ~TurtlebotDemo();
    
    bool init();
    void run();
    void enable();
    void disable();
    double getAngularVelocity();

    // Keyboard stuff
    int getChr();
    void keyboardThread();

private:
    ros::Publisher motor_power_publisher;
    ros::Publisher velocity_publisher;

    const double angular_vel = 0.2;
    double current_angle = 0.0;
    double max_angle = 2.5, min_angle = -2.5;
    bool turning_left = true;

    bool motor_power_status = false;
    bool connected = false;

    geometry_msgs::Twist cmd;

    bool paused = false;
    bool running = false;
    ecl::Thread kbd_thread;
};

}

#endif