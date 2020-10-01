#include "../include/gc2_turtlebot_demo.h"

namespace gc2_demo {

    TurtlebotDemo::TurtlebotDemo() {
        std::cout << "Creating turtlebot." << std::endl;
    }

    TurtlebotDemo::~TurtlebotDemo() {
        std::cout << "Destroying turtlebot." << std::endl;
    }

    bool TurtlebotDemo::init() {
        std::cout << "Init turtlebot." << std::endl;

        ros::NodeHandle nh("~");
        
        name = nh.getUnresolvedNamespace();

        nh.getParam("angular_vel_step", angular_vel_step);
        nh.getParam("angular_vel_max", angular_vel_max);

        return true;
    }

    void TurtlebotDemo::run() {
        std::cout << "Running demo..." << std::endl;
    }

}