#include "../include/gc2_turtlebot_demo.h"

namespace gc2_demo {

    TurtlebotDemo::TurtlebotDemo() {
        std::cout << "Creating turtlebot." << std::endl;
    }

    TurtlebotDemo::~TurtlebotDemo() {
        std::cout << "Destroying turtlebot." << std::endl;
        disable();
    }

    bool TurtlebotDemo::init() {
        std::cout << "Init turtlebot." << std::endl;

        ros::NodeHandle nh("~");
        
        //auto name = nh.getUnresolvedNamespace();

        motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

        // Connect to kobuki base
        connected = false;
        uint8_t count = 0;
        while(!connected && count <= 5) {
            if(motor_power_publisher.getNumSubscribers() > 0) {
                connected = true;
                break;
            }
            count++;
            usleep(1000000);
        }

        if(connected) {
            motor_power_status = true;
            kobuki_msgs::MotorPower power_cmd;
            power_cmd.state = kobuki_msgs::MotorPower::ON;
            motor_power_publisher.publish(power_cmd);
            ROS_INFO("GC2 demo connected to kobuki base.");
        }

        current_angle = 0.0;
        return connected;
    }

    void TurtlebotDemo::run() {
        std::cout << "Running demo..." << std::endl;

        enable();

        ros::Rate loop_rate(10);

        // Spawn keyboard thread
        running = true;
        kbd_thread.start(&gc2_demo::TurtlebotDemo::keyboardThread, *this);

        uint8_t count = 0;
        cmd.angular.z = getAngularVelocity();
        while(running && ros::ok()) {
            // Send velocity command if not paused
            if(!paused) {
                if(count >= 5) {
                    cmd.angular.z = getAngularVelocity();
                    count = 0;
                    ROS_INFO("Rotating with vel: %.2f current angle: %.2f", cmd.angular.z, current_angle);
                }
                velocity_publisher.publish(cmd);
                ros::spinOnce();
                loop_rate.sleep();
                count += 1;
            }

            // Check on thread, if its dead quit the application
            if(!kbd_thread.isRunning()) {
                running = false;
                disable(); // kill motor
            }
        }
        
        running = false;
        kbd_thread.join();

        disable();
    }

    void TurtlebotDemo::enable() {
        cmd.angular.z = 0;
        velocity_publisher.publish(cmd);

        if(!motor_power_status) {
            ROS_INFO("Enabling motor power.");
            kobuki_msgs::MotorPower power_cmd;
            power_cmd.state = kobuki_msgs::MotorPower::ON;
            motor_power_publisher.publish(power_cmd);
            motor_power_status = true;
        } else {
            ROS_WARN("Motors already enabled.");
        }

    }

    void TurtlebotDemo::disable() {
        cmd.angular.z = 0;
        velocity_publisher.publish(cmd);

        if(motor_power_status) {
            ROS_INFO("Disabling motor power.");
            kobuki_msgs::MotorPower power_cmd;
            power_cmd.state = kobuki_msgs::MotorPower::OFF;
            motor_power_publisher.publish(power_cmd);
            motor_power_status = false;
        } else {
            ROS_WARN("Motors already disabled");
        }
    }

    double TurtlebotDemo::getAngularVelocity() {
        if(turning_left) {
            if(current_angle >= max_angle) turning_left = false;
        } else {
            if(current_angle <= min_angle) turning_left = true;
        }
        double step = turning_left ? angular_vel : -angular_vel;
        current_angle += step;
        return step;
    }

    
    int TurtlebotDemo::getChr() {
        static struct termios oldt, newt;
        tcgetattr( STDIN_FILENO, &oldt); 
        newt = oldt;
        newt.c_lflag &= ~(ICANON);           
        tcsetattr( STDIN_FILENO, TCSANOW, &newt); 

        int c = getchar(); 

        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }

    void TurtlebotDemo::keyboardThread() {
        while(running) {
            int c = getChr();
            switch(c) {
                case 32: //space
                    ROS_INFO("SPACE PRESSED");
                    paused = !paused;
                    break;
                default:
                    break;
            }
            usleep(500000);
        }
    }
}