#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <atomic>
#include <cstddef>
#include <mutex>
#include <thread>

#include "sandbox/base_visualization.h"

class BulletEngine;
class DrawableMultiBody;
class MPProblem;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// Simulate an MPProblem using the bullet physics engine. Rendering is
/// performed by the base_visualization parent class.
///
/// @note This is a singleton object; there can only be one instance.
////////////////////////////////////////////////////////////////////////////////
class Simulation : public base_visualization {

  private:

    ///@name Construction
    ///@{

    /// Create a simulation of an MPProblem.
    /// @param[in] _problem The MPProblem to simulate.
    /// @param[in] _edit Start in edit mode instead of the physical simulator?
    Simulation(MPProblem* const _problem, const bool _edit);

    virtual ~Simulation();

  public:

    /// Create the singleton.
    /// @param[in] _problem The MPProblem to simulate.
    /// @param[in] _edit Start in edit mode instead of the physical simulator?
    static void Create(MPProblem* const _problem, const bool _edit = false);

    /// Get the singleton.
    static Simulation* Get();

    ///@}
    ///@name Simulation Interface
    ///@{

    /// Set up the simulation.
    void Initialize();

    /// Tear down the simulation.
    void Uninitialize();

    /// Advance the simulation one timestep.
    void SimulationStep();

    /// Update the drawable transforms to match those in the MPProblem.
    void EditStep();

    ///@}
    ///@name Visualization Interface
    ///@{

    virtual void render() override;
    virtual void start() override;
    virtual void reset() override;

    /// Set the maximum number of frames that the simulator can pre-compute.
    /// @param _max The maximum number of backlogged frames to compute.
    void SetBacklog(const size_t _max);

    ///@}
    ///@name Locking
    ///@{
    /// Edit tools may require the simulation to halt while model data is
    /// changed. This should not be used anywhere else.

    void Lock();
    void Unlock();

    ///@}
    ///@name Editing
    ///@{

    /// Rebuild the physics engine's model for a given multibody.
    /// @param _m The drawable multibody to rebuild.
    /// @WARNING The simulation must be locked while you do this!
    void RebuildMultiBody(DrawableMultiBody* const _m);

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
    size_t m_backlogMax{100};              ///< Max number of precomputed steps.

    const bool m_editMode{false};          ///< Are we in edit mode?

    ///@}
    ///@name Deleted Functions
    ///@{
    /// Move/copy are not supported.

    Simulation(const Simulation&) = delete;
    Simulation(Simulation&&)      = delete;

    Simulation& operator=(const Simulation&) = delete;
    Simulation& operator=(Simulation&&)      = delete;

    ///@}

};

#endif
