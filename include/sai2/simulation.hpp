// Simulation.hpp

#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <Eigen/Dense>
#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "Sai2Common.h"

using namespace Eigen;
using namespace std;

class Simulation
{
public:
    // Constructor
    Simulation(shared_ptr<Sai2Model::Sai2Model> robot,
               shared_ptr<Sai2Simulation::Sai2Simulation> sim,
               std::mutex &mutex_simulation,
               double sim_freq = 500.0);

    // Destructor
    ~Simulation();

    // Start the simulation loop in a separate thread
    void start();

    // Stop the simulation loop
    void stop();

    void updateTourque(const Vector<double, 7> &torque);
    void updateJointPosition(const Vector<double, 7> q);

private:
    // Simulation loop function
    void simulationLoop();

    shared_ptr<Sai2Model::Sai2Model> robot_;
    shared_ptr<Sai2Simulation::Sai2Simulation> sim_;
    unique_ptr<Sai2Common::LoopTimer> timer_;
    thread thread_;
    atomic<bool> fSimulationRunning_;
    Vector<double, 7> joint_torque_, joint_position_;
    Vector<double, 7> ui_torques_;
    std::mutex &mutex_simulation_;

    // Parameters for simulation
    double _sim_freq;
    string robot_name_;
    Eigen::Vector<double, 7> q_;
};

#endif // SIMULATION_HPP
