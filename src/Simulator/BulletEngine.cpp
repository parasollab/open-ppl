#include "BulletEngine.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "ConvexDecomposition/cd_wavefront.h"

#include "nonstd/runtime.h"

#include "Conversions.h"

#include "Geometry/Bodies/ActiveMultiBody.h"
#include "Geometry/Bodies/FixedBody.h"
#include "Geometry/Bodies/FreeBody.h"
#include "Geometry/Bodies/StaticMultiBody.h"


/*------------------------------ Construction --------------------------------*/

BulletEngine::
BulletEngine() {
  // Create the bullet objects needed for a dynamic rigid body simulation.
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_broadphase = new btDbvtBroadphase();
  m_solver = new btMultiBodyConstraintSolver();

  m_dynamicsWorld = new btMultiBodyDynamicsWorld(m_dispatcher,
      m_broadphase, m_solver, m_collisionConfiguration);

  // This is needed to get gimpact shapes to respond to collisions.
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  // Set the gravity in our world.
  m_dynamicsWorld->setGravity(btVector3(0,-10, 0));
}


BulletEngine::
~BulletEngine() {
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

  // Collision shapes aren't deleted with their associated rigid bodies because
  // multiple rigid bodies might share a collision shape. Delete them explicitly.
  for(int i = 0; i < m_collisionShapes.size(); ++i)
    delete m_collisionShapes[i];
  m_collisionShapes.clear();

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
Step(const btScalar _timestep, const int _maxSubSteps,
    const btScalar _resolution) {
  // This doesn't seem to be required in the examples, testing without it for
  // now.
  //m_dynamicsWorld->updateAabbs();
  //m_dynamicsWorld->computeOverlappingPairs();
  m_dynamicsWorld->stepSimulation(_timestep, _maxSubSteps, _resolution);
}

/*----------------------------- Transform Access -----------------------------*/

glutils::transform
BulletEngine::
GetObjectTransform(const size_t _i) const {
  // Check for out-of-range access.
  nonstd::assert_msg(_i < size_t(m_dynamicsWorld->getNumMultibodies()),
      "BulletEngine error: requested transform for object " + std::to_string(_i)
      + ", but there are only " +
      std::to_string(m_dynamicsWorld->getNumMultibodies()) +
      " objects in the simulation.");

  glutils::transform t;
  btMultiBody* mb = m_dynamicsWorld->getMultiBody(_i);
  mb->getBaseWorldTransform().getOpenGLMatrix(t.data());

  return t;
}

/*----------------------------- Modifiers ------------------------------------*/

btMultiBody*
BulletEngine::
AddObject(MultiBody* _m) {
  // Get base transform and mass.
  btTransform trans;
  double mass;

  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_m);
  if(sbody) {
    auto body = sbody->GetFixedBody(0);
    trans = ToBullet(body->GetWorldTransformation());
    mass = 0;
  }
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_m);
    auto body = abody->GetFreeBody(0);
    trans = ToBullet(body->GetWorldTransformation());
    mass = 1; ///< @TODO change the mass to appropriate amount
  }

  btCollisionShape* shape = BuildCollisionShape(_m);
  return AddObject(shape, trans, mass);
}


btMultiBody*
BulletEngine::
AddObject(btCollisionShape* _shape, const btTransform& _trans, double _mass) {
  // Add the shape to a list of shapes to be deleted later.
  m_collisionShapes.push_back(_shape);

  // Compute inertia.
  btVector3 inertia(0, 0, 0);
  const bool isDynamic(_mass != 0.);
  if(isDynamic)
    _shape->calculateLocalInertia(_mass, inertia);

  // Each object will be represented by a bullet rigid body, and each rigid body
  // has a motion state to describe it's current position, velocity, etc.
  //btDefaultMotionState* state = new btDefaultMotionState(_trans);
  //btRigidBody::btRigidBodyConstructionInfo rbInfo(_mass, state, _shape, inertia);
  //btRigidBody* body = new btRigidBody(rbInfo);
  // Add the rigid body to the bullet world.
  //m_dynamicsWorld->addRigidBody(body);

  // Make multi body.
  const int links = 0; // Number of links in addition to the base.
  const bool fixedBase = !isDynamic; // Base is fixed?
  const bool canSleep = false; // Can this object sleep? Not sure what it means.
  btMultiBody* mb = new btMultiBody(links, _mass, inertia, fixedBase, canSleep);
  mb->finalizeMultiDof();
  mb->setBaseWorldTransform(_trans);
  mb->setBaseVel(btVector3(0, 0, 0));

  // The multibody on its own doesn't process collisions. Add a collider for
  // each link. The base collider has link id -1.
  btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(mb, -1);
  col->setCollisionShape(_shape);
  col->setWorldTransform(_trans);

  m_dynamicsWorld->addMultiBody(mb);
  // Not sure what the numbers mean here but we need them. Appears to be related
  // to collisions being checked in groups, see
  // BulletCollision/CollisionDispatch/btCollisionWorld.h
  m_dynamicsWorld->addCollisionObject(col, 2, 1 + 2);
  mb->setBaseCollider(col);

  return mb;
}

/*------------------------------ Helpers -------------------------------------*/

btCollisionShape*
BulletEngine::
BuildCollisionShape(MultiBody* _body) {
  // Get the multibody's obj file and use it to create a bullet body.
  /// @TODO Parse all components of the multibodies instead of just the first.
  ///       Look at bullet/examples/MultiBody/MultiDofDemo.cpp
  ///           and bullet/src/BulletDynamics/FeatherStone/btMultiBody.h
  std::string filename;

  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_body);
  if(sbody)
    filename = sbody->GetFixedBody(0)->GetFilePath();
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_body);
    filename = abody->GetFreeBody(0)->GetFilePath();
  }

  // Make simulator collision shape.
  auto ptr = new btGImpactMeshShape(BuildCollisionShape(filename));
  ptr->updateBound();

  return ptr;
}


btTriangleMesh*
BulletEngine::
BuildCollisionShape(const std::string& _filename) {
  ConvexDecomposition::WavefrontObj obj;
  obj.loadObj(_filename.c_str());

  // Build a btTriangleMesh from an obj file.
  btTriangleMesh* mesh = new btTriangleMesh();
  mesh->preallocateVertices(obj.mVertexCount);
  mesh->preallocateIndices(obj.mTriCount);

  // Add vertices.
  for(int i = 0; i < obj.mVertexCount; ++i) {
    int start = i * 3;
    mesh->findOrAddVertex(btVector3(obj.mVertices[start],
                                    obj.mVertices[start + 1],
                                    obj.mVertices[start + 2]), false);
  }

  // Add facets.
  for(int i = 0; i < obj.mTriCount; ++i) {
    int start = i * 3;
    mesh->addTriangleIndices(obj.mIndices[start],
                             obj.mIndices[start + 1],
                             obj.mIndices[start + 2]);
  }

  return mesh;
}

/*----------------------------------------------------------------------------*/
