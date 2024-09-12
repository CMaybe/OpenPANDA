// main.cpp

#include <Sai2Graphics.h>
#include <Sai2Model.h>
#include <Sai2Simulation.h>
#include <signal.h>

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

#include "robot/panda_controller.hpp"
#include "sai2/control.hpp"
#include "sai2/simulation.hpp"

using namespace std;
using namespace Eigen;

// Global variables
atomic<bool> fSimulationRunning(true);
// Signal handler to stop threads
void signalHandler(int)
{
    fSimulationRunning = false;
}

int main()
{
    // File paths and robot name
    const string world_file = "resources/world.urdf";
    const string robot_file = "resources/model/panda_arm.urdf";
    const string robot_name = "PANDA";
    const string link_name = "end-effector";
    const std::string robot_ip = "172.16.0.2";
    const std::string gripper_ip = "172.16.0.2";

    std::cout << "Press Enter to start..." << std::endl;
    std::cin.ignore();

    MatrixXd Jv, Jv_T;
    MatrixXd linear_manipulability;
    Eigen::Vector3d ellipsoid;

    mutex mutex_control, mutex_simulation, mutex_panda;
    Vector<double, 7> ui_torques;
    Vector<double, 7> control_torques;

    // Set up signal handler
    signal(SIGABRT, &signalHandler);
    signal(SIGTERM, &signalHandler);
    signal(SIGINT, &signalHandler);

    // Load graphics scene
    auto graphics = make_shared<Sai2Graphics::Sai2Graphics>(world_file);
    graphics->addUIForceInteraction(robot_name);

    // Load simulation world
    auto sim = make_shared<Sai2Simulation::Sai2Simulation>(world_file);

    // Load robots
    auto robot = make_shared<Sai2Model::Sai2Model>(robot_file, false);
    robot->setQ(sim->getJointPositions(robot_name));
    robot->updateModel();

    std::shared_ptr<chai3d::cMultiPoint> manipulability_cloud = make_shared<chai3d::cMultiPoint>();
    graphics->_world->addChild(manipulability_cloud.get());

    // Initialize torques
    ui_torques = Vector<double, 7>::Zero(robot->dof());
    control_torques = Vector<double, 7>::Zero(robot->dof());

    // Create Simulation and Control objects
    Simulation simulation(robot, sim, mutex_simulation, 10.0);
    simulation.start();

    // comment for just simulation
    PandaController panda_controller(robot_ip, gripper_ip, mutex_panda);
    panda_controller.start();

    // uncomment for just simulation
    // Control control(robot, sim, mutex_control, mutex_simulation);
    // control.start();

    while (graphics->isWindowOpen())
    {
        {
            lock_guard<mutex> lock1(mutex_simulation);
            manipulability_cloud->clear();
            Vector3d robot_pose = robot->position(link_name);
            Jv = robot->Jv(link_name);
            Jv_T = Jv.transpose();
            linear_manipulability = Jv * Jv_T;
            for (double i = 0; i <= M_PI; i += 0.1)
            {
                for (double j = 0; j < 2 * M_PI; j += 0.15)
                {
                    Eigen::Vector3d dir_vec(sin(i) * cos(j), sin(i) * sin(j), cos(i));
                    ellipsoid = linear_manipulability * dir_vec;
                    manipulability_cloud->newPoint(chai3d::cVector3d(robot_pose + ellipsoid));
                }
            }
            manipulability_cloud->setPointColor(chai3d::cColorf(0, 0, 255, 0.5));
        }
        {
            lock_guard<mutex> lock(mutex_simulation);
            graphics->updateRobotGraphics(robot_name, robot->q());
        }
        graphics->renderGraphicsWorld();
        {
            ui_torques = graphics->getUITorques(robot_name);
        }
        {
            // comment for just simulation
            simulation.updateJointPosition(panda_controller.getRobotJointPosition());

            // uncomment for just simulation
            // simulation.updateTourque(control.getTorque());
        }
    }

    // Stop simulation and control threads
    fSimulationRunning = false;
    simulation.stop();
    panda_controller.stop();
    // control.stop();

    return 0;
}