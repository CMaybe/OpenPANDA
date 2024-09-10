// Control.hpp

#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "timer/LoopTimer.h"
#include "logger/Logger.h"

using namespace Eigen;
using namespace std;

class Control
{
public:
    // Constructor
    Control(shared_ptr<Sai2Model::Sai2Model> robot,
            shared_ptr<Sai2Simulation::Sai2Simulation> sim,
            std::mutex &mutex_control,
            std::mutex &mutex_simulation,
            double control_freq = 1000.0);

    ~Control();

    // Start the control loop in a separate thread
    void start();

    // Stop the control loop
    void stop();

    Vector<double, 7> getTorque() const;

private:
    // Control loop function
    void controlLoop();

    shared_ptr<Sai2Model::Sai2Model> robot_;
    shared_ptr<Sai2Simulation::Sai2Simulation> sim_;
    unique_ptr<Sai2Common::LoopTimer> timer_;
    shared_ptr<Sai2Primitives::MotionForceTask> motion_force_task_;
    shared_ptr<Sai2Primitives::JointTask> joint_task_;
    thread thread_;
    atomic<bool> fControlRunning_;
    Vector<double, 7> control_torques_;
    Vector<double, 7> ui_torques_;
    Vector<double, 7> initial_q_;

    std::mutex &mutex_control_, &mutex_simulation_;

    // Control parameters
    double control_freq_;
    string robot_name_;
    int dof_;
};

#endif // CONTROL_HPP
