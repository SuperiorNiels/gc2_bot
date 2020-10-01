#include "../include/gc2_turtlebot_demo.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "gc2_turtlebot_demo");
    gc2_demo::TurtlebotDemo gc2();
    
    if(gc2.init()) gc2.run();
    else ROS_ERROR_STREAM("Could not init gc2 demo properly!");

    return 0;
}