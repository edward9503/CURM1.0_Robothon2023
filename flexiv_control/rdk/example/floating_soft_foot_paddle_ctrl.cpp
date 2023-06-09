/**
 * @example floating_with_soft_limits.cpp
 * Run damped floating with soft limits enabled to keep joints from
 * hitting position limits.
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */

#include <flexiv/Robot.hpp>
#include <flexiv/Exception.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <string>
#include <thread>

namespace {
/** Robot DOF */
const int k_robotDofs = 7;

/** Damping gains for floating */
const std::vector<double> k_floatingDamping
    = {10.0, 10.0, 5.0, 5.0, 1.0, 1.0, 1.0};
}


/** IO Init */
bool cur_value = 0;
bool last_value = 0;

/** Callback function for realtime periodic task */
void periodicTask(flexiv::Robot* robot, flexiv::Scheduler* scheduler,
    flexiv::Log* log, flexiv::RobotStates& robotStates)
{
    try {
        // Monitor fault on robot server
        if (robot->isFault()) {
            throw flexiv::ServerException(
                "periodicTask: Fault occurred on robot server, exiting ...");
        }

        // Read robot states
        robot->getRobotStates(robotStates);

        // Read IO states
        cur_value = robot->readDigitalInput(0);
        std::cout << cur_value << std::endl;

        
        // Set 0 joint torques
        std::vector<double> torqueDesired(k_robotDofs, 0.0);

        // Add some velocity damping
    
        if(cur_value == 1) {
            robot->setMode(flexiv::MODE_JOINT_TORQUE);
            for (size_t i = 0; i < k_robotDofs; ++i) {
                torqueDesired[i] = -k_floatingDamping[i] * robotStates.dtheta[i];               
            }
            robot->streamJointTorque(torqueDesired, true, true);

        }else{
            //torqueDesired[i] = -20*k_floatingDamping[i] * robotStates.dtheta[i];
            robot->stop();
        }
        
        // Send target joint torque to RDK server, enable gravity compensation
        // and joint soft limits
        //robot->streamJointTorque(torqueDesired, true, true);


        /*if(cur_value!=last_value) {
            std::cout << cur_value << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            cur_value = robot->readDigitalInput(0);
            if(cur_value!=last_value) {
                last_value = cur_value;
                if(cur_value == 0) {
                    // Set 0 joint torques
                    std::vector<double> torqueDesired(k_robotDofs, 0.0);

                    // Add some velocity damping
                    for (size_t i = 0; i < k_robotDofs; ++i) {
                        torqueDesired[i] = -k_floatingDamping[i] * robotStates.dtheta[i];
                    }

                    // Send target joint torque to RDK server, enable gravity compensation
                    // and joint soft limits
                    robot->streamJointTorque(torqueDesired, true, true);
                }
            }
        }
        */
    } catch (const flexiv::Exception& e) {
        log->error(e.what());
        scheduler->stop();
    }
}

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

int main(int argc, char* argv[])
{
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

        robot.stop();
        // Set mode after robot is operational
        // robot.setMode(flexiv::MODE_JOINT_TORQUE);
        
        // Wait for the mode to be switched
        
        // while (robot.getMode() != flexiv::MODE_JOINT_TORQUE) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
        
        // Periodic Tasks
        //=============================================================================
        flexiv::Scheduler scheduler;
        // Add periodic task with 1ms interval and highest applicable priority
        scheduler.addTask(
            std::bind(periodicTask, &robot, &scheduler, &log, robotStates),
            "HP periodic", 1, 45);
        // Start all added tasks, this is by default a blocking method
        scheduler.start();

    } catch (const flexiv::Exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
