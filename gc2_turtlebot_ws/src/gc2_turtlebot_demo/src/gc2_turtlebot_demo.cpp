#include "../include/gc2_turtlebot_demo.h"

namespace gc2_demo {

    TurtlebotDemo() {
        std::cout << "Creating turtlebot." << std::endl;
    }

    ~TurtlebotDemo() {
        std::cout << "Destroying turtlebot." << std::endl;
    }

    bool TurtlebotDemo::init() {
        std::cout << "Init turtlebot." << std::endl;
        return true;
    }

    void TurtlebotDemo::run() {
        std::cout << "Running demo..." << std::endl;
    }

}