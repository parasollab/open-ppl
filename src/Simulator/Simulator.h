#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "btBulletDynamicsCommon.h"
#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/MPProblem.h"
#include "ConvexDecomposition/cd_wavefront.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "Conversions.h"

#include <iostream>

class MultiBody;


btTriangleMesh* build_triangle(const ConvexDecomposition::WavefrontObj& _obj);


template<class T>
class Simulator {

  public:

    ///@name Construction
    ///@{

    Simulator(T* _problem);
    ~Simulator();

    ///@}
    ///@name Simulation Interface
    ///@{

    void Initialize();
    void Step();

    ///@}

  private:

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Create a bullet body a multi body and add it to the dynamics
    ///        world
    void AddWorldObject(MultiBody* _body);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Add an object to the world
    /// @param _shape the bullet collision shape that reprsents the object
    /// @param _trans the world tranform of the object
    /// @param _mass the mass of the _shape in the world, if 0 object is static
    ///              otherwise it is a dynamic object
    void AddWorldObject(btCollisionShape* _shape, const btTransform& _trans,
        double _mass);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Create a btCollisionShape from the environment bounding box and
    ///        set the environment bounding box member variable
    void SetEnvBBX();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Adds all environment obstacles to the world
    ///          all obstacles will by static for now
    void AddObstacles();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Adds all of the robots to the world
    ///        all robots will be dynamic
    void AddRobots();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Builds a collision shape from a multibody
    btCollisionShape* BuildCollisionShape(MultiBody* _body);


    T* m_problem; ///< MPProblem object

    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btBroadphaseInterface* m_overlappingPairCache;
    btSequentialImpulseConstraintSolver* m_solver;

    btDynamicsWorld* m_dynamicsWorld;                           ///< dynamics world
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;  ///< list of all objects in the world
};

/*---------------------------- Construction ----------------------------------*/

template<class T>
Simulator<T>::
Simulator(T* _problem) : m_problem(_problem) {}


template<class T>
Simulator<T>::
~Simulator() {
  int i;
  for(i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
      btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);

      if(body && body->getMotionState())
        delete body->getMotionState();

      m_dynamicsWorld->removeCollisionObject(obj);
      delete obj;
  }

  for(i = 0; i < m_collisionShapes.size(); ++i) {
    btCollisionShape* shape = m_collisionShapes[i];
    m_collisionShapes[i] = nullptr;
    delete shape;
  }

  delete m_dynamicsWorld;
  delete m_solver;
  delete m_overlappingPairCache;
  delete m_dispatcher;

  m_collisionShapes.clear();
}


template<class T>
void
Simulator<T>::
Initialize() {
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_overlappingPairCache = new btDbvtBroadphase();
  m_solver = new btSequentialImpulseConstraintSolver;
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
      m_overlappingPairCache, m_solver, m_collisionConfiguration);

  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);

  m_dynamicsWorld->setGravity(btVector3(0,-10, 0));

  if(m_problem) {
    // Set environment bounding box
    SetEnvBBX();

    // add obstacle objects to the world
    AddObstacles();

    // add robot objects to the world
    AddRobots();
  }
}


template<class T>
void
Simulator<T>::
Step() {
  m_dynamicsWorld->updateAabbs();
  m_dynamicsWorld->computeOverlappingPairs();

  // steps the simulation of the world by a set delta time
  m_dynamicsWorld->stepSimulation(1.f/60.f, 10);

  // print positions of all objects
  //
  for(int j = m_dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; --j) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
    btRigidBody* body = btRigidBody::upcast(obj);

    btTransform trans;

    if(body && body->getMotionState())
      body->getMotionState()->getWorldTransform(trans);
    else
      trans = obj->getWorldTransform();

    cout << "World pos object " << j << " = " << trans.getOrigin().getX()
      << ", " << trans.getOrigin().getY() << ", "
      << trans.getOrigin().getZ() << endl;
  }
  cout << endl;
}


template<class T>
void
Simulator<T>::
AddWorldObject(MultiBody* _body) {
  // TODO: make an accurate representation of the object
}


template<class T>
void
Simulator<T>::
SetEnvBBX() {
  const auto& boundary = m_problem->GetEnvironment()->GetBoundary();

  // Get bounding box ranges
  auto r1 = boundary->GetRange(0);
  auto r2 = boundary->GetRange(1);
  auto r3 = boundary->GetRange(2);

  // Compute half edge lengths of bounding box
  double x = (r1.second - r1.first) / 2;
  double y = (r2.second - r2.first) / 2;
  double z = (r3.second - r3.first) / 2;

  // Create a btCollisionShape using half lengths of the bounding box
  btCollisionShape* bbx = new btBoxShape(btVector3(btScalar(x), btScalar(1.),
      btScalar(z)));
  AddWorldObject(bbx, btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -y, 0)),
      0.);
}


template<class T>
void
Simulator<T>::
AddWorldObject(btCollisionShape* _shape, const btTransform& _trans,
    double _mass) {
  // add the shape to a list of shapes to be deleted later
  m_collisionShapes.push_back(_shape);

  btTransform trans = _trans;

  btScalar mass(_mass);

  // if the mass of the object is 0 then the object is static
  // otherwise it is dynamic
  bool isDynamic(mass != 0.f);

  btVector3 localInertia(0,0,0);

  // if the body is dynamic then get the local inertial
  if(isDynamic)
    _shape->calculateLocalInertia(mass, localInertia);

  // creating bullet dynamics objects needed by the world to simulate an
  // objects motion
  btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, _shape,
      localInertia);
  btRigidBody* body = new btRigidBody(rbInfo);


  // add the rigid body to the dynamics world
  m_dynamicsWorld->addRigidBody(body);
}


template<class T>
void
Simulator<T>::
AddObstacles() {
  Environment* env = m_problem->GetEnvironment();

  for(size_t i = 0; i < env->NumObstacles(); ++i) {
    StaticMultiBody* body = env->GetObstacle(i);
    btCollisionShape* temp = BuildCollisionShape(body);
    AddWorldObject(temp,
        ToBullet(body->GetFixedBody(0)->GetWorldTransformation()), 0.);

    btTransform trans;
    trans.setIdentity();

    btVector3 min, max;

    temp->getAabb(trans, min, max);

    cout << min << endl << max << endl;
  }
}

template<class T>
void
Simulator<T>::
AddRobots() {
  /// @TODO fix to get World Transform properly
  /// @TODO change the mass to appropriate amount
  for(size_t i = 0; i < m_problem->NumRobots(); ++i) {
    ActiveMultiBody* body = m_problem->GetRobot(i);
    AddWorldObject(BuildCollisionShape(body),
        ToBullet(body->GetFreeBody(0)->GetWorldTransformation()), 1);
  }
}


template<class T>
btCollisionShape*
Simulator<T>::
BuildCollisionShape(MultiBody* _body) {
  // Processing the multibody's obj file.
  ConvexDecomposition::WavefrontObj wfo;

  StaticMultiBody* sbody = dynamic_cast<StaticMultiBody*>(_body);
  if(sbody)
    wfo.loadObj(sbody->GetFixedBody(0)->GetFilePath().c_str());
  else {
    ActiveMultiBody* abody = dynamic_cast<ActiveMultiBody*>(_body);
    wfo.loadObj(abody->GetFreeBody(0)->GetFilePath().c_str());
  }

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
