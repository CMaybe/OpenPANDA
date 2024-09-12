#include "sai2/simulation.hpp"

Simulation::Simulation(shared_ptr<Sai2Model::Sai2Model> robot,
                       shared_ptr<Sai2Simulation::Sai2Simulation> sim,
                       std::mutex &mutex_simulation,
                       double sim_freq)
    : robot_(robot), sim_(sim), mutex_simulation_(mutex_simulation), _sim_freq(sim_freq), fSimulationRunning_(true)
{
    timer_ = make_unique<Sai2Common::LoopTimer>(_sim_freq);
    robot_name_ = "PANDA";  // Adjust this based on your actual robot name
}

Simulation::~Simulation()
{
    stop();  // Ensure that the thread is stopped before destruction
}

void Simulation::start()
{
    fSimulationRunning_ = true;
    thread_ = std::thread(&Simulation::simulationLoop, this);
}

void Simulation::stop()
{
    fSimulationRunning_ = false;
    if (thread_.joinable())
    {
        thread_.join();  // Wait for the thread to finish execution
    }
}

void Simulation::updateTourque(const Vector<double, 7> &torque)
{
    lock_guard<mutex> lock(mutex_simulation_);
    this->joint_torque_ = torque;
}

void Simulation::updateJointPosition(const Vector<double, 7> q)
{
    lock_guard<mutex> lock(mutex_simulation_);
    this->q_ = q;
}

void Simulation::simulationLoop()
{
    sim_->setTimestep(1.0 / _sim_freq);

    while (fSimulationRunning_)
    {
        timer_->waitForNextLoop();
        {
            lock_guard<mutex> lock(mutex_simulation_);
            // sim_->setJointTorques(robot_name_, joint_torque_);
            sim_->setJointPositions(robot_name_, q_);
            robot_->setQ(sim_->getJointPositions(robot_name_));
            robot_->updateModel();
            sim_->integrate();
        }
    }
    timer_->stop();
    timer_->printInfoPostRun();
}
