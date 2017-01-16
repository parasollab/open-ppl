#include "Simulation.h"

#include <iostream>

#include "BulletEngine.h"
#include "Drawable.h"
#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"

#include "nonstd/io.h"
#include "nonstd/runtime.h"


/*---------------------------- Construction ----------------------------------*/

Simulation::
Simulation(MPProblem* const _problem) : m_problem(_problem) {}


Simulation::
~Simulation() {
  reset();
}

/*-------------------------- Simulation Interface ----------------------------*/

void
Simulation::
Initialize() {
  // If the engine object is not null, we are already initialized.
  if(m_engine)
    return;

  // Require a non-null problem to initialize.
  nonstd::assert_msg(m_problem, "Simulation error: cannot initialize with a "
      "null problem!");

  // Create a bullet engine.
  m_engine = new BulletEngine;

  // Add the problem objects to the simulation.
  AddBBX();
  AddRobots();
  AddObstacles();
}


void
Simulation::
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


void
Simulation::
Step() {
  // Push the current transforms for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    if(m_backloggedSteps > m_backlogMax)
      return;
    ++m_backloggedSteps;

    for(size_t i = 0; i < this->m_drawables.size(); ++i) {
      /// @TODO Fix this to the transform of object i once we make the bbx
      ///       drawable.
      auto d = static_cast<Drawable*>(this->m_drawables[i]);
      //d->PushTransform(0, m_engine->GetObjectTransform(i + 1, 0));
      for(size_t j = 0; j < d->GetNumBodies(); ++j)
        d->PushTransform(j, m_engine->GetObjectTransform(i + 1, j));
    }
  }

  // Step the simulation forward with a fixed timestep.
  static constexpr btScalar timestep = 2.f / 60.f;   // Advance by this much...
  static constexpr btScalar resolution = 1.f / 60.f; // Using tics this long...
  static constexpr int maxSubSteps = 2;              // Up to this many ticks.
  m_engine->Step(timestep, maxSubSteps, resolution);

  // Step each Robot's agent.
  for(size_t i = 0; i < m_problem->NumRobots(); ++i)
    m_problem->GetNewRobot(i)->Step(timestep);
}

/*------------------------ Visualization Interface ---------------------------*/

void
Simulation::
render() {
  // Update the transform for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    // If there are no queued frames, there is nothing new to render. Leave the
    // previous frame on the scene.
    if(m_backloggedSteps == 0)
      return;
    --m_backloggedSteps;

    // Otherwise, update objects to the next queued positions.
    for(auto d : m_drawables)
      static_cast<Drawable*>(d)->UpdateTransform();
  }

  // Rrrrrender.
  base_visualization::render();
}


void
Simulation::
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


void
Simulation::
reset() {
  if(!m_running)
    return;
  m_running = false;

  // Wait for the worker thread to stop before uninitializing.
  m_worker.join();

  Uninitialize();
}

/*----------------------------------- Helpers --------------------------------*/

void
Simulation::
AddBBX() {
  const auto& boundary = m_problem->GetEnvironment()->GetBoundary();

  // Get bounding box ranges
  auto r1 = boundary->GetRange(0);
  auto r2 = boundary->GetRange(1);
  auto r3 = boundary->GetRange(2);

  // Compute half edge lengths of bounding box
  const btScalar thickness = 1.;
  const btScalar x = r1.Center();
  const btScalar y = r2.min - thickness / 2;
  const btScalar z = r3.Center();

  // Make the transform.
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(btVector3(0, -y, 0));

  // Create a btCollisionShape using half lengths of the bounding box
  btBoxShape* box = new btBoxShape(btVector3(x, thickness, z));
  m_engine->AddObject(box, trans, 0.);

  /// @TODO Add visual for the bounding box/floor.
  /// @TODO Support other boundary types.
  //auto d = new Drawable(?);
  //this->add_drawable(d);
}


void
Simulation::
AddObstacles() {
  Environment* env = m_problem->GetEnvironment();

  for(size_t i = 0; i < env->NumObstacles(); ++i) {
    MultiBody* body = env->GetObstacle(i);
    m_engine->AddObject(body);
    this->add_drawable(new Drawable(body));
  }
}


void
Simulation::
AddRobots() {
  for(size_t i = 0; i < m_problem->NumRobots(); ++i) {
    MultiBody* body = m_problem->GetRobot(i);
    auto bulletModel = m_engine->AddObject(body);
    m_problem->GetNewRobot(i)->SetDynamicsModel(bulletModel);
    this->add_drawable(new Drawable(body));
  }
}

/*----------------------------------------------------------------------------*/
