#include "sai2/simulation.hpp"

Simulation::Simulation(shared_ptr<Sai2Model::Sai2Model> robot,
                       shared_ptr<Sai2Simulation::Sai2Simulation> sim,
                       std::mutex &mutex_simulation,
                       double sim_freq)
    : _robot(robot), _sim(sim), _mutex_simulation(mutex_simulation), _sim_freq(sim_freq), _fSimulationRunning(true)
{
    _timer = make_unique<Sai2Common::LoopTimer>(_sim_freq);
    _robot_name = "PANDA"; // Adjust this based on your actual robot name
}

Simulation::~Simulation()
{
    stop(); // Ensure that the thread is stopped before destruction
}

void Simulation::start()
{
    _fSimulationRunning = true;
    _thread = std::thread(&Simulation::simulationLoop, this);
}

void Simulation::stop()
{
    _fSimulationRunning = false;
    if (_thread.joinable())
    {
        _thread.join(); // Wait for the thread to finish execution
    }
}

void Simulation::updateTourque(const Vector<double, 7> &torque)
{
    lock_guard<mutex> lock(_mutex_simulation);
    this->_joint_torque = torque;
}

void Simulation::updateJointPosition(const Vector<double, 7> q)
{
    lock_guard<mutex> lock(_mutex_simulation);
    this->q_ = q;
}

void Simulation::simulationLoop()
{
    _sim->setTimestep(1.0 / _sim_freq);

    while (_fSimulationRunning)
    {
        _timer->waitForNextLoop();
        {
            lock_guard<mutex> lock(_mutex_simulation);
            // _sim->setJointTorques(_robot_name, _joint_torque);
            _sim->setJointPositions(_robot_name, q_);
            _robot->setQ(_sim->getJointPositions(_robot_name));
            _robot->updateModel();
            _sim->integrate();
        }
    }
    _timer->stop();
    _timer->printInfoPostRun();
}
