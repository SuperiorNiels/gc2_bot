#ifndef GC2_TURTLEBOT_DEMO
#define GC2_TURTLEBOT_DEMO

#include <ros/ros.h>
#include <termios.h>
#include <iostream>

#include <kobuki_msgs/MotorPower.h>

namespace gc2_demo {

class TurtlebotDemo {
public:
    TurtlebotDemo();
    ~TurtlebotDemo();
    bool init();

    void run();

};

}

#endif