/**
 * @author Shengzhi Wang
 */

// Ros-related
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <thread>

std_msgs::String arm_primitive_cmd;

void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot IP] [local IP]" << std::endl;
    std::cout << "    robot IP: address of the robot server" << std::endl;
    std::cout << "    local IP: address of this PC" << std::endl;
    std::cout << "Optional arguments: None" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

void armPrimitiveCmdCallback(const std_msgs::String& msg)
{
    arm_primitive_cmd.data = msg.data;
}

int main(int argc, char* argv[])
{
    // ROS initialization
    ros::init(argc, argv, "flexiv_control_loop_contact_mode");
    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::NodeHandle n;
    ros::Rate rate(1000.0);

    ros::Publisher arm_task_status_pub = n.advertise<std_msgs::String>("/arm_task_status", 1);
    ros::Publisher arm_primitive_cmd_pub = n.advertise<std_msgs::String>("/arm_primitive_cmd", 1);  // This is specifically used for cleaning the topic data
    ros::Subscriber arm_primitive_cmd_sub = n.subscribe("/arm_primitive_cmd", 1, armPrimitiveCmdCallback);

    // Log object for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse Parameters
    //=============================================================================
    if (argc < 3
        || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }

    // IP of the robot server
    std::string robotIP = argv[1];

    // IP of the workstation PC running this program
    std::string localIP = argv[2];

    try {
        // RDK Initialization
        //=============================================================================
        // Instantiate robot interface
        flexiv::Robot robot(robotIP, localIP);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        int secondsWaited = 0;
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++secondsWaited == 10) {
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode");
            }
        }
        log.info("Robot is now operational");

        // Set mode after robot is operational
        robot.setMode(flexiv::MODE_PRIMITIVE_EXECUTION);

        // Wait for the mode to be switched
        while (robot.getMode() != flexiv::MODE_PRIMITIVE_EXECUTION) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Define empty_msg
        std_msgs::String empty_msg;
        empty_msg.data = "";

        // Define last_arm_primitive_cmd to check whether a new data is subscribed from topic
        std_msgs::String last_arm_primitive_cmd;
        last_arm_primitive_cmd.data = empty_msg.data;
        arm_primitive_cmd.data = empty_msg.data;

        // Define arm_task_status_msg
        std_msgs::String arm_task_status_msg;
        arm_task_status_msg.data = "Done.";

        // Clean the topic before entering the while loop
        arm_task_status_pub.publish(empty_msg);
        arm_primitive_cmd_pub.publish(empty_msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // while (ros::ok()){
        //     if (last_arm_primitive_cmd.data != arm_primitive_cmd.data){
        //         std::cout << "I am not equal!!!\n";

        // Get the robot states
        robot.getRobotStates(robotStates);
        auto start_tcpPose = robotStates.tcpPose;

                // Send command to robot
                // robot.executePrimitive(arm_primitive_cmd.data);
                robot.executePrimitive("CaliForceSensor()");
                std::this_thread::sleep_for(std::chrono::seconds(5));
                std::cout << "finish calibrate!!!\n";
                // robot.executePrimitive("MoveJ(target=131 7 56 120 -3 25 -132, maxVel=0.07)");
                // robot.executePrimitive("AlignContact(maxVel=0.1)");
                robot.executePrimitive("SlideSpiral(contactForce=5)");
                // // Wait for reached target
                // while (flexiv::utility::parsePtStates(robot.getPrimitiveStates(), "alignContacted")=="False" & ros::ok()) {
                //     std::this_thread::sleep_for(std::chrono::seconds(1));
                //     std::cout << flexiv::utility::parsePtStates(robot.getPrimitiveStates(), "alignContacted") << "\n";
                // }
                while (ros::ok() 
                    & flexiv::utility::parsePtStates(robot.getPrimitiveStates(), "pushDistance") < "0.019") {
                    robot.getRobotStates(robotStates);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    std::cout << "distance error: \n" << start_tcpPose[2] - robotStates.tcpPose[2] << "\n";
                }

                std::cout << "I am done!!!\n";
                // arm_primitive_cmd.data = empty_msg.data;
                // last_arm_primitive_cmd.data = empty_msg.data;
                // arm_task_status_pub.publish(arm_task_status_msg);
                // arm_primitive_cmd_pub.publish(arm_primitive_cmd);
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            // }else{
            //     // std::cout << "I am equal!!!\n";
            // }

        //     ros::spinOnce();
        //     rate.sleep();
        // }

        // All done, stop robot and put into IDLE mode, then stop the spinner for ros
        robot.stop();
        spinner.stop();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        spinner.stop();
        return 1;
    }

    return 0;
}
