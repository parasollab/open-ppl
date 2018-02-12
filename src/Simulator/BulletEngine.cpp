#include "BulletEngine.h"

#include "BulletModel.h"
#include "Conversions.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/GMSPolyhedron.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/Robot/DynamicsModel.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/runtime.h"


/*--------------------------- Static Initialization --------------------------*/

std::map<btDynamicsWorld*, BulletEngine::CallbackSet&>
BulletEngine::s_callbackMap;

/*------------------------------ Construction --------------------------------*/

BulletEngine::
BulletEngine(MPProblem* const _problem) : m_problem(_problem) {
  // Create the bullet objects needed for a dynamic rigid body simulation.
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();
  m_solver = new btMultiBodyConstraintSolver();

  m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
      m_broadphase, m_solver, m_collisionConfiguration);

  // This is needed to get gimpact shapes to respond to collisions.
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  // Set the gravity in our world, based off of MPProblem:
  m_dynamicsWorld->setGravity(ToBullet(
                              m_problem->GetEnvironment()->GetGravity()));

  // Add this engine's call-back set to the call-back map.
  s_callbackMap.emplace(m_dynamicsWorld, m_callbacks);
  m_dynamicsWorld->setInternalTickCallback(ExecuteCallbacks);
}


BulletEngine::
~BulletEngine() {
  // Acquire the lock for the remainder of object life.
  std::lock_guard<std::mutex> lock(m_lock);

  // Delete the rigid bodies in the bullet dynamics world.
  for(int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);

    // If the body has a motion state, delete that too.
    if(body && body->getMotionState())
      delete body->getMotionState();

    m_dynamicsWorld->removeCollisionObject(obj);
    delete obj;
  }

  // Clear the models before the dynamicsWorld as the former need the later to
  // tear down properly.
  m_models.clear();

  // Remove this engine's dynamics world and call-back set from the call-back
  // map.
  s_callbackMap.erase(m_dynamicsWorld);

  // Delete the remaining bullet objects.
  delete m_dynamicsWorld;
  delete m_solver;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_collisionConfiguration;
}

/*---------------------------- Simulation Interface --------------------------*/

void
BulletEngine::
Step(const btScalar _timestep) {
  std::lock_guard<std::mutex> lock(m_lock);

  // Rebuild any objects which need it.
  RebuildObjects();

  // This doesn't seem to be required in the examples, testing without it for
  // now.
  //m_dynamicsWorld->updateAabbs();
  //m_dynamicsWorld->computeOverlappingPairs();

  // Advance the simulation by '_timestep' units using up to 'maxSubSteps' sub
  // steps of length 'resolution'.
  const btScalar resolution = m_problem->GetEnvironment()->GetTimeRes();
  const int maxSubSteps = std::ceil(_timestep / resolution);

  m_dynamicsWorld->stepSimulation(_timestep, maxSubSteps, resolution);
}

/*----------------------------- Transform Access -----------------------------*/

glutils::transform
BulletEngine::
GetObjectTransform(const size_t _i, const size_t _j) const {
  // Check for out-of-range access.
  nonstd::assert_msg(_i < size_t(m_dynamicsWorld->getNumMultibodies()),
      "BulletEngine error: requested transform for object " + std::to_string(_i)
      + ", but there are only " +
      std::to_string(m_dynamicsWorld->getNumMultibodies()) +
      " objects in the simulation.");

  btMultiBody* mb = m_dynamicsWorld->getMultiBody(_i);

  std::array<double, 16> buffer;
  if(_j == 0)
    mb->getBaseWorldTransform().getOpenGLMatrix(buffer.data());
  else
    mb->getLink(int(_j-1)).m_cachedWorldTransform.getOpenGLMatrix(buffer.data());

  /// @TODO Fix this to avoid the extra copy.
  glutils::transform t;
  std::copy(buffer.begin(), buffer.end(), t.begin());

  return t;
}

/*----------------------------- Modifiers ------------------------------------*/

btMultiBody*
BulletEngine::
AddRobot(Robot* const _robot) {
  // This is an active body.
  MultiBody* const multiBody = _robot->GetMultiBody();

  // We need to set the body at its zero Cfg to create the bullet model. Save
  // the current DOFs so that we can restore them afterward.
  std::vector<double> startDofs = multiBody->GetCurrentDOFs();
  multiBody->Configure(std::vector<double>(startDofs.size(), 0));

  btMultiBody* const bulletModel = AddObject(multiBody);

  // Restore the original configuration.
  multiBody->Configure(startDofs);

  // Configure the bullet body to match.
  Cfg cfg(_robot);
  cfg.SetData(startDofs);
  ConfigureSimulatedState(cfg, bulletModel);

  // Set the bullet model as the robot's dynamics model.
  _robot->SetDynamicsModel(bulletModel);

  // If the robot is car-like, create the necessary callbacks.
  if(_robot->IsCarlike())
    CreateCarlikeCallback(bulletModel);

  return bulletModel;
}


btMultiBody*
BulletEngine::
AddObject(MultiBody* const _m) {
  std::lock_guard<std::mutex> lock(m_lock);

  // Check that we haven't already added this model.
  if(m_models.count(_m))
    throw RunTimeException(WHERE, "Cannot add the same MultiBody twice.");

  m_models[_m] = std::unique_ptr<BulletModel>(new BulletModel(_m));
  m_models[_m]->Initialize();
  m_models[_m]->AddToDynamicsWorld(m_dynamicsWorld);

  return m_models[_m]->GetBulletMultiBody();
}


void
BulletEngine::
SetGravity(const btVector3& _gravity) {
  std::lock_guard<std::mutex> lock(m_lock);

  m_dynamicsWorld->setGravity(_gravity);
}


void
BulletEngine::
RebuildObject(MultiBody* const _m) {
  std::lock_guard<std::mutex> lock(m_lock);

  m_rebuildQueue.push(_m);
}

/*--------------------------------- Helpers ----------------------------------*/

void
BulletEngine::
RebuildObjects() {
  while(!m_rebuildQueue.empty()) {
    // Dequeue the next multibody.
    MultiBody* const m = m_rebuildQueue.front();
    m_rebuildQueue.pop();

    // Check that this model exists.
    auto iter = m_models.find(m);
    if(iter == m_models.end())
      throw RunTimeException(WHERE, "MultiBody not found.");

    // Rebuild the model.
    iter->second->Rebuild();
  }
}

/*--------------------------- Callback Functions -----------------------------*/

void
BulletEngine::
CreateCarlikeCallback(btMultiBody* const _model) {
  // Create a call-back function to make _model appear car-like. For now we will
  // always assume that _model's forward direction is (1, 0, 0) in its local
  // frame.
  CallbackFunction f = [_model]() {
    const btVector3 velocity = _model->getBaseVel();
    const btVector3 heading = _model->localDirToWorld(-1, {1, 0, 0});
    const btScalar sign = velocity * heading < 0 ? -1 : 1;
    _model->setBaseVel(heading * sign * velocity.length());
  };

  // Add this to the set of callbacks for this simulation engine.
  m_callbacks.push_back(f);
}


void
BulletEngine::
ExecuteCallbacks(btDynamicsWorld* _world, btScalar _timeStep) {
  // Get the call-back set associated with this dynamics world.
  CallbackSet& callbacks = s_callbackMap.at(_world);

  // Execute each call-back in the set.
  for(auto& callback : callbacks)
    callback();
}

/*----------------------------------------------------------------------------*/
