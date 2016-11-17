#include "BulletEngine.h"

#include "Conversions.h"

#include "ConvexDecomposition/cd_wavefront.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#include "nonstd/runtime.h"

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
  m_overlappingPairCache = new btDbvtBroadphase();
  m_solver = new btSequentialImpulseConstraintSolver();

  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
      m_overlappingPairCache, m_solver, m_collisionConfiguration);

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
  delete m_overlappingPairCache;
  delete m_dispatcher;
  delete m_collisionConfiguration;
}

/*---------------------------- Simulation Interface --------------------------*/

void
BulletEngine::
Step(const btScalar _timestep, const int _maxSubSteps,
    const btScalar _resolution) {
  m_dynamicsWorld->updateAabbs();
  m_dynamicsWorld->computeOverlappingPairs();
  m_dynamicsWorld->stepSimulation(_timestep, _maxSubSteps, _resolution);
}

/*----------------------------- Transform Access -----------------------------*/

glutils::transform
BulletEngine::
GetObjectTransform(const size_t _i) const {
  // Check for out-of-range access.
  nonstd::assert_msg(_i < size_t(m_dynamicsWorld->getNumCollisionObjects()),
      "BulletEngine error: requested transform for object " + std::to_string(_i)
      + ", but there are only " +
      std::to_string(m_dynamicsWorld->getNumCollisionObjects()) +
      " objects in the simulation.");

  glutils::transform t;

  // Get object _i and try casting to rigid body.
  btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[_i];
  btRigidBody* body = btRigidBody::upcast(obj);

  if(body && body->getMotionState()) {
    // This is a movable object. Get transform from the motion state.
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(t.data());
  }
  else {
    // This is a static object. Get transform directly from the object.
    obj->getWorldTransform().getOpenGLMatrix(t.data());
  }

  return t;
}

/*----------------------------- Modifiers ------------------------------------*/

void
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
  AddObject(shape, trans, mass);
}


void
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
  btDefaultMotionState* state = new btDefaultMotionState(_trans);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(_mass, state, _shape, inertia);
  btRigidBody* body = new btRigidBody(rbInfo);

  // Add the rigid body to the bullet world.
  m_dynamicsWorld->addRigidBody(body);
}

/*------------------------------ Helpers -------------------------------------*/

btCollisionShape*
BulletEngine::
BuildCollisionShape(MultiBody* _body) {
  // Get the multibody's obj file and use it to create a bullet body.
  /// @TODO Parse all components of the multibodies instead of just the first.
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
