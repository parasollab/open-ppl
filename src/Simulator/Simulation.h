#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>

#include "sandbox/base_visualization.h"

class BulletEngine;
class MPProblem;


////////////////////////////////////////////////////////////////////////////////
/// Simulate an MPProblem using the bullet physics engine. Rendering is
/// performed by the base_visualization parent class.
////////////////////////////////////////////////////////////////////////////////
class Simulation : public base_visualization {

  public:

    ///@name Construction
    ///@{

    /// Create a simulation of an MPProblem.
    /// @param[in] _problem The MPProblem to simulate.
    Simulation(MPProblem* const _problem);

    virtual ~Simulation();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Set up the simulation.
    void Initialize();

    /// Tear down the simulation.
    void Uninitialize();

    /// Advance the simulation one timestep.
    void Step();

    ///@}
    ///@name Visualization Interface
    ///@{

    virtual void render() override;
    virtual void start() override;
    virtual void reset() override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Create a btCollisionShape for the environment bounding box.
    void AddBBX();

    /// Add all environment obstacles to the bullet world.
    void AddObstacles();

    /// Add all robots in the problem to the bullet world.
    void AddRobots();

    ///@}
    ///@name Internal State
    ///@{

    MPProblem* const m_problem;        ///< The MPProblem we are simulating.
    BulletEngine* m_engine{nullptr};   ///< An engine to drive the simulation.

    mutable std::mutex m_guard;        ///< Lock for updating object transforms.

    std::thread m_worker;                  ///< Thread for stepping simulation.
    std::atomic<bool> m_running{false};    ///< Is the simulation running?

    volatile size_t m_backloggedSteps{0};  ///< Number of precomputed steps.
    const size_t m_backlogMax{100};        ///< Max number of precomputed steps.

    ///@}

};

#endif
