#include "../include/gc2_turtlebot_demo.h"


namespace gc2_demo {

    TurtlebotDemo::TurtlebotDemo() {}

    TurtlebotDemo::~TurtlebotDemo() {
        disable();
    }

    bool TurtlebotDemo::init() {
        std::cout << "Init turtlebot." << std::endl;

        ros::NodeHandle nh('~');

        // Get parameters from ros
        ros::param::get("~angular_vel", angular_vel);
        ros::param::get("~turn_time_s", turn_time_s);
        ros::param::get("~cool_time_s", cool_time_s);
        
        //auto name = nh.getUnresolvedNamespace();

        motor_power_publisher = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);

        command_subscriber = nh.subscribe("/robot/action", 1, &TurtlebotDemo::cmdCallback, this);

        control_timer = nh.createTimer(ros::Duration(0.1), &TurtlebotDemo::controlCallback, this);

        // Connect to kobuki base
        connected = false;
        uint8_t count = 0;
        while(!connected && count <= 5) {
            if(motor_power_publisher.getNumSubscribers() > 0) {
                connected = true;
                break;
            }
            count++;
            usleep(1000000); // sleep 1s
        }

        if(connected) {
            motor_power_status = true;
            kobuki_msgs::MotorPower power_cmd;
            power_cmd.state = kobuki_msgs::MotorPower::ON;
            motor_power_publisher.publish(power_cmd);
            ROS_INFO("GC2 demo connected to kobuki base.");
        }

        return connected;
    }

    void TurtlebotDemo::controlCallback(const ros::TimerEvent& e) {
        // If prev time is not initialized, skip (for more accurate timing)
        if(!state_mutex.try_lock()) {
            ROS_WARN("Timer missed: state locked.");
            state_mutex.unlock();
            return;
        }
        if(current_state.last_checked.isZero()) {
            current_state.current_action = TURNING_LEFT;
            current_state.start_time = ros::Time::now();
            current_state.action_time = ros::Duration(turn_time_s);
            current_state.last_checked = ros::Time::now();
            return;
        }

        ros::Duration elapsed = ros::Time::now() - current_state.start_time;
        //ROS_INFO("Elapsed time: %f ms", elapsed.toSec());

        switch(current_state.current_action) {
            case TURNING_LEFT:
                cmd.angular.z = -angular_vel;
                velocity_publisher.publish(cmd);
                if(elapsed >= current_state.action_time) {
                    // Action time succeeded, change state
                    current_state.current_action = COOLDOWN;
                    current_state.prev_action = TURNING_LEFT;
                    current_state.action_time = ros::Duration(cool_time_s);
                    current_state.start_time = ros::Time::now();
                    ROS_INFO("state: %s time: %.3fs.", action_strings[current_state.current_action], current_state.action_time.toSec());
                }
                break;
            case TURNING_RIGHT:
                cmd.angular.z = angular_vel;
                velocity_publisher.publish(cmd);
                if(elapsed >= current_state.action_time) {
                    // Action time succeeded, change state
                    current_state.current_action = COOLDOWN;
                    current_state.prev_action = TURNING_RIGHT;                    
                    current_state.action_time = ros::Duration(cool_time_s);
                    current_state.start_time = ros::Time::now();
                    ROS_INFO("state: %s time: %.3fs.", action_strings[current_state.current_action], current_state.action_time.toSec());
                }
                break;
            case COOLDOWN:
                if(elapsed >= current_state.action_time) {
                    // Action time succeeded, change state
                    current_state.current_action = current_state.prev_action == TURNING_LEFT ? TURNING_RIGHT : TURNING_LEFT;
                    current_state.prev_action = COOLDOWN;
                    current_state.action_time = ros::Duration(turn_time_s);
                    current_state.start_time = ros::Time::now();
                    ROS_INFO("state: %s time: %.3fs.", action_strings[current_state.current_action], current_state.action_time.toSec());
                }
            case PAUSED:
            default:
                // Sending zero command should not be necessary here (avoid spamming controller)
                //cmd.angular.z = 0;
                //velocity_publisher.publish(cmd);
                break;
        }
        
        current_state.last_checked = ros::Time::now();
        state_mutex.unlock();

        // Check on thread, if its dead quit the application
        if(!kbd_thread.isRunning()) {
            running = false;
            disable(); // kill motor
        }
    }

    void TurtlebotDemo::run() {
        // Spawn keyboard thread
        running = true;
        kbd_thread.start(&gc2_demo::TurtlebotDemo::keyboardThread, *this);
        
        // Ros main loop, and enable motors
        enable();
        ros::spin();
        disable();

        // Program finished, stop thread
        running = false;
        kbd_thread.join();
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

    void TurtlebotDemo::updatePaused() {
        state_mutex.lock(); // blocking call, wait until timer cb is done
                    
        if(current_state.current_action == PAUSED) {
            // Resume prev action
            current_state.current_action = current_state.prev_action;
            current_state.prev_action = PAUSED;
            current_state.start_time = ros::Time::now();
            ROS_INFO("Resuming previous action for: %.3fs.", current_state.action_time.toSec());
        } else {
            ROS_INFO("Pausing current action.");
            // Pause
            if(current_state.current_action == COOLDOWN) {
                // If cooldown period, perform state switch and set paused mode
                current_state.current_action = current_state.prev_action == TURNING_LEFT ? TURNING_RIGHT : TURNING_LEFT;;
                current_state.prev_action = COOLDOWN;
                current_state.action_time = ros::Duration(turn_time_s);
                current_state.start_time = ros::Time::now();
            }
            current_state.prev_action = current_state.current_action;
            current_state.current_action = PAUSED;
            current_state.action_time = current_state.action_time - (ros::Time::now() - current_state.start_time);
        }

        state_mutex.unlock();
    }

    void TurtlebotDemo::cmdCallback(const std_msgs::String::ConstPtr& msg) {
        if(msg->data.find("go") != std::string::npos) {
            if(current_state.current_action == PAUSED) updatePaused();
        } else if(msg->data.find("stop") != std::string::npos) {
            if(current_state.current_action != PAUSED) updatePaused();
        } else {
            ROS_WARN("Received unknown external command.");
        }
    }
    
    int TurtlebotDemo::getChar() {
        static struct termios oldt, newt;
        tcgetattr( STDIN_FILENO, &oldt); 
        newt = oldt;
        newt.c_lflag &= ~(ICANON);    
        newt.c_cc[VMIN] = 0;
        newt.c_cc[VTIME] = 0; // non-blocking mode      
        tcsetattr( STDIN_FILENO, TCSANOW, &newt); 

        int c = getchar(); 

        tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }

    void TurtlebotDemo::keyboardThread() {
        while(running) {
            int c = getChar();
            switch(c) {
                case 32: //space
                    updatePaused();
                    break;
                default:
                    break;
            }
            usleep(10000); // sleep 10ms
        }
    }
}