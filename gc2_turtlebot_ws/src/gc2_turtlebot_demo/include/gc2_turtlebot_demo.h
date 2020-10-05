#ifndef GC2_TURTLEBOT_DEMO
#define GC2_TURTLEBOT_DEMO

#include <iostream>
#include <string>
#include <mutex>

#include <ros/ros.h>
#include <termios.h>
#include <ecl/time.hpp>
#include <ecl/threads.hpp>

#include <kobuki_msgs/MotorPower.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>

#define PI 3.14159265359

namespace gc2_demo {

enum action {
    TURNING_LEFT,
    TURNING_RIGHT,
    COOLDOWN,
    PAUSED
};

const static char *action_strings[PAUSED + 1] = {"TURNING_LEFT", "TURNING_RIGHT", "COOLDOWN", "PAUSED"};

typedef struct {
    action current_action;
    action prev_action;         // store prev action (so we can resume after pause)
    ros::Duration action_time;  // time the current action is supposed to take
    ros::Time start_time;       // time the current action started
    ros::Time last_checked;     // last time the values where updated
} demo_state;

class TurtlebotDemo {
public:
    TurtlebotDemo();
    ~TurtlebotDemo();
    
    bool init();
    void run();
    void enable();
    void disable();
    void controlCallback(const ros::TimerEvent& e); // ran every 100 ms (by timer)
    void updatePaused();
    void cmdCallback(const std_msgs::String::ConstPtr& msg);

    // Keyboard stuff
    int getChar();
    void keyboardThread();

private:
    ros::Publisher motor_power_publisher;
    ros::Publisher velocity_publisher;

    ros::Subscriber command_subscriber;

    ros::Timer control_timer;
    demo_state current_state;
    std::mutex state_mutex;

    const double angular_vel = 0.5; // 0.5 rad/s
    const double turn_time_s = 5.0; // 5s turing
    const double cool_time_s = 1.0; // 1s cooldown

    bool motor_power_status = false;
    bool connected = false;

    geometry_msgs::Twist cmd;

    bool paused = false;
    bool running = false;
    ecl::Thread kbd_thread;
};

}

#endif