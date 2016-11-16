#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/MPProblem.h"

#include "btBulletDynamicsCommon.h"
#include "ConvexDecomposition/cd_wavefront.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

#include "sandbox/gui/base_visualization.h"
#include "nonstd/io.h"

#include "Drawable.h"
#include "Conversions.h"

#include <iostream>


btTriangleMesh* build_triangle(const ConvexDecomposition::WavefrontObj& _obj);


////////////////////////////////////////////////////////////////////////////////
/// Simulate an MPProblem using the bullet physics engine. Rendering is
/// performed by the base_visualization parent class.
////////////////////////////////////////////////////////////////////////////////
template <typename MPProblemType>
class Simulator : public base_visualization {

  public:

    ///@name Construction
    ///@{

    Simulator(MPProblemType* _problem);
    virtual ~Simulator();

    ///@}
    ///@name Simulation Interface
    ///@{

    void Initialize();
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

    /// Create a bullet body a multi body and add it to the dynamics world.
    void AddWorldObject(MultiBody* _body);

    /// Add an object to the world.
    /// @param _shape The bullet collision shape that reprsents the object.
    /// @param _trans The world tranform of the object.
    /// @param _mass The mass of the _shape in the world.
    void AddWorldObject(btCollisionShape* _shape, const btTransform& _trans,
        double _mass);

    /// Create a btCollisionShape from the environment bounding box.
    void SetEnvBBX();

    /// Add all environment obstacles to the bullet world.
    void AddObstacles();

    /// Add all robots in the problem to the bullet world.
    void AddRobots();

    /// Build a bullet collision shape from a pmpl MultiBody.
    /// @param[in] _body The pmpl MultiBody.
    btCollisionShape* BuildCollisionShape(MultiBody* _body);

    ///@}
    ///@name Internal State
    ///@{

    MPProblemType* m_problem; ///< The MPProblem we are simulating.

    btDefaultCollisionConfiguration* m_collisionConfiguration{nullptr};
    btCollisionDispatcher* m_dispatcher{nullptr};
    btBroadphaseInterface* m_overlappingPairCache{nullptr};
    btSequentialImpulseConstraintSolver* m_solver{nullptr};

    /// The bullet world.
    btDynamicsWorld* m_dynamicsWorld{nullptr};

    /// The collision shapes in our bullet world.
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    mutable std::mutex m_guard; ///< Lock object data for step or render.

    ///@}
};

/*---------------------------- Construction ----------------------------------*/

template <typename MPProblemType>
Simulator<MPProblemType>::
Simulator(MPProblemType* _problem) : m_problem(_problem) {}


template <typename MPProblemType>
Simulator<MPProblemType>::
~Simulator() {
  // Delete the rigid bodies in the bullet dynamics world.
  for(int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);

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

  delete m_dynamicsWorld;
  delete m_solver;
  delete m_overlappingPairCache;
  delete m_dispatcher;
}

/*-------------------------- Simulation Interface ----------------------------*/

template <typename MPProblemType>
void
Simulator<MPProblemType>::
Initialize() {
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_overlappingPairCache = new btDbvtBroadphase();
  m_solver = new btSequentialImpulseConstraintSolver();

  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
      m_overlappingPairCache, m_solver, m_collisionConfiguration);

  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  m_dynamicsWorld->setGravity(btVector3(0,-10, 0));

  if(m_problem) {
    SetEnvBBX();
    AddRobots();
    AddObstacles();
  }
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
Step() {
  // Update collision object bounding boxes and compute overlaps.
  m_dynamicsWorld->updateAabbs();
  m_dynamicsWorld->computeOverlappingPairs();

  // Step the simulation forward with a fixed timestep.
  static constexpr btScalar timestep = 2.f / 60.f;   // Advance by this much...
  static constexpr btScalar resolution = 1.f / 60.f; // Using tics this long...
  static constexpr int maxSubSteps = 2;              // Up to this many ticks.
  m_dynamicsWorld->stepSimulation(timestep, maxSubSteps, resolution);

  // Push the new transforms for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);

    for(int j = 1; j < m_dynamicsWorld->getNumCollisionObjects(); ++j) {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);

      btTransform trans;

      if(body && body->getMotionState()) {
        // This is a movable object. Update its rendering state.
        body->getMotionState()->getWorldTransform(trans);

        glutils::transform t;
        trans.getOpenGLMatrix(t.data());
        this->m_drawables[j - 1]->push_transform(t);
      }
      //else
      //  trans = obj->getWorldTransform();

      // Print for temporary debug.
      //cout << "World pos object " << j << " = " << trans.getOrigin().getX()
      //     << ", " << trans.getOrigin().getY() << ", "
      //     << trans.getOrigin().getZ() << endl;
    }
  }
  //cout << endl;
}

/*------------------------ Visualization Interface ---------------------------*/

template <typename MPProblemType>
void
Simulator<MPProblemType>::
render() {
  // Update the transform for all rendering objects.
  {
    std::lock_guard<std::mutex> lock(m_guard);
    for(const auto d : m_drawables)
      d->update_transform();
  }

  // Rrrrrender.
  base_visualization::render();
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
start() {
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
reset() {
}

/*-------------------------- Simulation Interface ----------------------------*/

template <typename MPProblemType>
void
Simulator<MPProblemType>::
AddWorldObject(MultiBody* _body) {
  /// @TODO Generalize bodies so that we can use this prototype.
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
SetEnvBBX() {
  const auto& boundary = m_problem->GetEnvironment()->GetBoundary();

  // Get bounding box ranges
  auto r1 = boundary->GetRange(0);
  auto r2 = boundary->GetRange(1);
  auto r3 = boundary->GetRange(2);

  // Compute half edge lengths of bounding box
  double thickness = 1.;
  double x = (r1.second - r1.first) / 2;
  double y = r2.first - thickness / 2;
  double z = (r3.second - r3.first) / 2;

  // Create a btCollisionShape using half lengths of the bounding box
  btBoxShape* bbx = new btBoxShape(btVector3(btScalar(x), btScalar(thickness),
      btScalar(z)));
  AddWorldObject(bbx, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -y, 0)),
      0.);

  //auto d = new Drawable(?);
  //this->add_drawable(d);
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
AddWorldObject(btCollisionShape* _shape, const btTransform& _trans,
    double _mass) {
  // Add the shape to a list of shapes to be deleted later.
  m_collisionShapes.push_back(_shape);

  // Compute local inertia.
  btVector3 localInertia(0, 0, 0);
  const bool isDynamic(_mass != 0.);
  if(isDynamic)
    _shape->calculateLocalInertia(_mass, localInertia);

  // Each object will be represented by a bullet rigid body, and each rigid body
  // has a motion state to describe it's current position, velocity, etc.
  btDefaultMotionState* myMotionState = new btDefaultMotionState(_trans);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(_mass, myMotionState, _shape,
      localInertia);
  btRigidBody* body = new btRigidBody(rbInfo);

  // Add the rigid body to the bullet world.
  m_dynamicsWorld->addRigidBody(body);
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
AddObstacles() {
  Environment* env = m_problem->GetEnvironment();

  for(size_t i = 0; i < env->NumObstacles(); ++i) {
    StaticMultiBody* body = env->GetObstacle(i);
    btCollisionShape* temp = BuildCollisionShape(body);
    AddWorldObject(temp,
        ToBullet(body->GetFixedBody(0)->GetWorldTransformation()), 0.);
  }
}


template <typename MPProblemType>
void
Simulator<MPProblemType>::
AddRobots() {
  /// @TODO change the mass to appropriate amount
  for(size_t i = 0; i < m_problem->NumRobots(); ++i) {
    ActiveMultiBody* body = m_problem->GetRobot(i);
    double mass = 1;
    AddWorldObject(BuildCollisionShape(body),
        ToBullet(body->GetFreeBody(0)->GetWorldTransformation()), mass);
  }
}


template <typename MPProblemType>
btCollisionShape*
Simulator<MPProblemType>::
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

  // Make rendering representation.
  auto d = new Drawable(filename);
  this->add_drawable(d);

  // Make simulator collision shape.
  ConvexDecomposition::WavefrontObj wfo;
  wfo.loadObj(filename.c_str());
  auto ptr = new btGImpactMeshShape(build_triangle(wfo));
  ptr->updateBound();

  return ptr;
}


btTriangleMesh*
build_triangle(const ConvexDecomposition::WavefrontObj& _obj) {
  // Build a btTriangleMesh from an obj file.
  btTriangleMesh* triangle = new btTriangleMesh();
  triangle->preallocateVertices(_obj.mVertexCount);
  triangle->preallocateIndices(_obj.mTriCount);

  // Add vertices.
  for(int i = 0; i < _obj.mVertexCount; ++i) {
    int start = i * 3;
    triangle->findOrAddVertex(btVector3(_obj.mVertices[start],
                                        _obj.mVertices[start + 1],
                                        _obj.mVertices[start + 2]), false);
  }

  // Add facets.
  for(int i = 0; i < _obj.mTriCount; ++i) {
    int start = i * 3;
    triangle->addTriangleIndices(_obj.mIndices[start],
                                 _obj.mIndices[start + 1],
                                 _obj.mIndices[start + 2]);
  }
  return triangle;
}

#endif
