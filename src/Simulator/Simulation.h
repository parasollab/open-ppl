#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "MPProblem/MPProblem.h"

#include "sandbox/gui/base_visualization.h"
#include "nonstd/io.h"

#include "BulletEngine.h"
#include "Drawable.h"

#include <cstddef>
#include <iostream>
#include <mutex>
#include <thread>


////////////////////////////////////////////////////////////////////////////////
/// Simulate an MPProblem using the bullet physics engine. Rendering is
/// performed by the base_visualization parent class.
////////////////////////////////////////////////////////////////////////////////
template <typename MPProblemType>
class Simulation : public base_visualization {

  public:

    ///@name Construction
    ///@{

    Simulation(MPProblemType* _problem);
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

    MPProblemType* m_problem{nullptr}; ///< The MPProblem we are simulating.
    BulletEngine* m_engine{nullptr};   ///< An engine to drive the simulation.

    mutable std::mutex m_guard; ///< Lock for updating object transforms.

    std::atomic<bool> m_running{false};    ///< Is the simulation running?
    volatile size_t m_backloggedSteps{0};  ///< Number of precomputed steps.
    const size_t m_backlogMax{100};        ///< Max number of precomputed steps.
    std::thread m_worker;                  ///< Thread for stepping simulation.

    ///@}
};

/*---------------------------- Construction ----------------------------------*/

template <typename MPProblemType>
Simulation<MPProblemType>::
Simulation(MPProblemType* _problem) : m_problem(_problem) {}


template <typename MPProblemType>
Simulation<MPProblemType>::
~Simulation() {
  reset();
}

/*-------------------------- Simulation Interface ----------------------------*/

template <typename MPProblemType>
void
Simulation<MPProblemType>::
Initialize() {
  // If the engine object is not null, we are already initialized.
  if(m_engine)
    return;

  // Require a non-null problem to initialize.
  nonstd::assert_msg(m_problem, "Simulation error: cannot initialize without a "
      "problem!");

  // Create a bullet engine.
  m_engine = new BulletEngine;

  // Add the problem objects to the simulation.
  AddBBX();
  AddRobots();
  AddObstacles();
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
Uninitialize() {
  // If the engine object is null, we are already uninitialized.
  if(!m_engine)
    return;

  // Delete the bullet engine.
  delete m_engine;
  m_engine = nullptr;

  // Delete the drawable objects.
  auto drawables = this->m_drawables;
  for(auto d : drawables)
    this->remove_drawable(d);

  // Remove bullet model pointers from problem objects.
  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetNewRobot(i)->SetDynamicsModel(nullptr);
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
Step() {
  // Push the current transforms for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    if(m_backloggedSteps > m_backlogMax)
      return;
    ++m_backloggedSteps;

    for(size_t i = 0; i < this->m_drawables.size(); ++i)
      /// @TODO Fix this to the transform of object i once we make the bbx
      ///       drawable.
      this->m_drawables[i]->push_transform(m_engine->GetObjectTransform(i + 1));
  }

  // Step the simulation forward with a fixed timestep.
  static constexpr btScalar timestep = 2.f / 60.f;   // Advance by this much...
  static constexpr btScalar resolution = 1.f / 60.f; // Using tics this long...
  static constexpr int maxSubSteps = 2;              // Up to this many ticks.
  m_engine->Step(timestep, maxSubSteps, resolution);

  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetNewRobot(i)->Step();
}

/*------------------------ Visualization Interface ---------------------------*/

template <typename MPProblemType>
void
Simulation<MPProblemType>::
render() {
  // Update the transform for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    if(m_backloggedSteps == 0)
      return;
    --m_backloggedSteps;

    for(const auto d : m_drawables)
      d->update_transform();
  }

  // Rrrrrender.
  base_visualization::render();
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
start() {
  if(m_running)
    return;
  m_running = true;

  Initialize();

  // Create a worker object to step the simulation.
  auto workFunction = [this]() {
    while(this->m_running)
      this->Step();
  };
  m_worker = std::thread(workFunction);
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
reset() {
  if(!m_running)
    return;
  m_running = false;

  // Wait for the worker thread to stop before uninitializing.
  m_worker.join();

  Uninitialize();
}

/*----------------------------------- Helpers --------------------------------*/

template <typename MPProblemType>
void
Simulation<MPProblemType>::
AddBBX() {
  const auto& boundary = m_problem->GetEnvironment()->GetBoundary();

  // Get bounding box ranges
  auto r1 = boundary->GetRange(0);
  auto r2 = boundary->GetRange(1);
  auto r3 = boundary->GetRange(2);

  // Compute half edge lengths of bounding box
  const btScalar thickness = 1.;
  const btScalar x = (r1.second - r1.first) / 2;
  const btScalar y = r2.first - thickness / 2;
  const btScalar z = (r3.second - r3.first) / 2;

  // Make the transform.
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(btVector3(0, -y, 0));

  // Create a btCollisionShape using half lengths of the bounding box
  btBoxShape* box = new btBoxShape(btVector3(x, thickness, z));
  m_engine->AddObject(box, trans, 0.);

  //auto d = new Drawable(?);
  //this->add_drawable(d);
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
AddObstacles() {
  Environment* env = m_problem->GetEnvironment();

  for(size_t i = 0; i < env->NumObstacles(); ++i) {
    MultiBody* body = env->GetObstacle(i);
    m_engine->AddObject(body);
    this->add_drawable(new Drawable(body));
  }
}


template <typename MPProblemType>
void
Simulation<MPProblemType>::
AddRobots() {
  for(size_t i = 0; i < m_problem->NumRobots(); ++i) {
    MultiBody* body = m_problem->GetRobot(i);
    auto bulletModel = m_engine->AddObject(body);
    m_problem->GetNewRobot(i)->SetDynamicsModel(bulletModel);
    this->add_drawable(new Drawable(body));
  }
}

/*----------------------------------------------------------------------------*/

#endif
