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

    shared_ptr<Sai2Model::Sai2Model> _robot;
    shared_ptr<Sai2Simulation::Sai2Simulation> _sim;
    unique_ptr<Sai2Common::LoopTimer> _timer;
    shared_ptr<Sai2Primitives::MotionForceTask> _motion_force_task;
    shared_ptr<Sai2Primitives::JointTask> _joint_task;
    thread _thread;
    atomic<bool> _fControlRunning;
    Vector<double, 7> _control_torques;
    Vector<double, 7> _ui_torques;
    Vector<double, 7> _initial_q;

    std::mutex &_mutex_control, &_mutex_simulation;

    // Control parameters
    double _control_freq;
    string _robot_name;
    int _dof;
};

#endif // CONTROL_HPP
