#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>

#include "sandbox/gui/base_visualization.h"

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

    Simulation(MPProblem* _problem);
    virtual ~Simulation();

    ///@}
    ///@name Simulation Interface
    ///@{

    void Initialize();
    void Uninitialize();
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

    MPProblem* m_problem{nullptr};     ///< The MPProblem we are simulating.
    BulletEngine* m_engine{nullptr};   ///< An engine to drive the simulation.

    mutable std::mutex m_guard;        ///< Lock for updating object transforms.

    std::atomic<bool> m_running{false};    ///< Is the simulation running?
    volatile size_t m_backloggedSteps{0};  ///< Number of precomputed steps.
    const size_t m_backlogMax{100};        ///< Max number of precomputed steps.
    std::thread m_worker;                  ///< Thread for stepping simulation.

    ///@}
};

#endif
