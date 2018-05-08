#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>

#include "sandbox/base_visualization.h"
#include "nonstd/collection.h"

class BulletEngine;
class Cfg;
class DrawableMultiBody;
class DrawablePath;
class MPProblem;
class MultiBody;
class StatClass;

/// @TODO Update gl_visualizer so that the main window is a singleton and
///       accessible via a static getter.
class main_window;
extern main_window* theOneWindow;


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
    Simulation(std::shared_ptr<MPProblem> _problem, const bool _edit);

  public:

    /// Create the singleton.
    /// @param[in] _problem The MPProblem to simulate.
    /// @param[in] _edit Start in edit mode instead of the physical simulator?
    static void Create(std::shared_ptr<MPProblem> _problem,
        const bool _edit = false);

    virtual ~Simulation();

    ///@}
    ///@name Accessors
    ///@{

    /// Get the singleton.
    static Simulation* Get() noexcept;

    /// Get the nearest number of timesteps needed to represent a continuous
    /// time interval.
    /// @param _dt The time interval.
    /// @return The number of steps needed to represent _dt.
    static size_t NearestNumSteps(const double _dt) noexcept;

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
    ///@name Rendering Interface
    ///@{

    virtual void render() override;
    virtual void start() override;
    virtual void reset() override;

    /// Set the maximum number of frames that the simulator can pre-compute.
    /// @param _max The maximum number of backlogged frames to compute.
    void SetBacklog(const size_t _max);

    ///@}
    ///@name Additional Visualization
    ///@{

    /// Draw a set of path edges as a series of connecting lines.
    /// @param _path The path to render.
    /// @param _c The line color.
    /// @return The ID of the path.
    size_t AddPath(const std::vector<Cfg>& _path, glutils::color _c);

    /// Remove a path from the scene.
    /// @param _id The path ID.
    void RemovePath(const size_t _id);

    ///@}
    ///@name Editing
    ///@{

    /// Rebuild the physics engine's model for a given multibody.
    /// @param _m The drawable multibody to rebuild.
    /// @WARNING The simulation will lock while you do this!
    void RebuildMultiBody(DrawableMultiBody* const _m);

    ///@}
    ///@name Performance Measurement
    ///@{

    /// Get the simulator's stat class for time profiling.
    static StatClass* GetStatClass() noexcept;

    /// Print the simulator's stats to an output file.
    /// @param _basename The base name for the output file, or empty to use the
    ///                  MPProblem base name.
    void PrintStatFile(const std::string& _basename = "");

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

    const std::shared_ptr<MPProblem> m_problem;  ///< The simulated MPProblem.
    BulletEngine* m_engine{nullptr};             ///< The physics engine.

    mutable std::mutex m_guard;            ///< Lock for updating transforms.
    std::thread m_worker;                  ///< Thread for stepping simulation.
    std::atomic<bool> m_running{false};    ///< Is the simulation running?
    volatile size_t m_backloggedSteps{0};  ///< Number of precomputed steps.
    size_t m_backlogMax{1};                ///< Max number of precomputed steps.

    const bool m_editMode{false};             ///< Are we in edit mode?

    std::unique_ptr<StatClass> m_stats;       ///< StatClass for time profiling.

    nonstd::collection<DrawablePath> m_paths; ///< Paths we are drawing.

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
