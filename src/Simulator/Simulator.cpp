#include "Simulator.h"
#include "Geometry/Bodies/MultiBody.h"
Simulator::
Simulator() {}

Simulator::
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

void 
Simulator::
Initialize() {
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_overlappingPairCache = new btDbvtBroadphase();
  m_solver = new btSequentialImpulseConstraintSolver;
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_overlappingPairCache,
                                              m_solver, m_collisionConfiguration);

  m_dynamicsWorld->setGravity(btVector3(0,-10, 0));

  {
    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));


    m_collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();

    groundTransform.setOrigin(btVector3(0, -56, 0));

    btScalar mass(0.);

    bool isDynamic(mass != 0.f);

    btVector3 localInertia(0,0,0);

    if(isDynamic)
      groundShape->calculateLocalInertia(mass, localInertia);


    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);

    btRigidBody* body = new btRigidBody(rbInfo);

    m_dynamicsWorld->addRigidBody(body);
  }

  {
    btCollisionShape* colShape = new btSphereShape(btScalar(1.));

    m_collisionShapes.push_back(colShape);

    btTransform startTransform;
    startTransform.setIdentity();


    btScalar mass(1.);

    bool isDynamic(mass != 0.f);

    btVector3 localInertia(0,0,0);

    if(isDynamic)
      colShape->calculateLocalInertia(mass, localInertia);



    startTransform.setOrigin(btVector3(2,10,0));

    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    m_dynamicsWorld->addRigidBody(body);
  }
}

void 
Simulator::
Step() {
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

      printf("World pos object %d = %lf, %lf, %lf \n", j, trans.getOrigin().getX(), trans.getOrigin().getY(),
      trans.getOrigin().getZ());
    }
}



void
Simulator::
AddWorldObject(MultiBody* _body) {
// the basic code for adding a rigid body to the world from a bullet collision
// shape. The multi body will have a bullet collision shape associated to it.
//
//    m_collisionShapes.push_back(colShape);
//
//    btTransform trans;
//    trans.setIdentity();
//
//
//    btScalar mass(_mass);
//
//    bool isDynamic(mass != 0.f);
//
//    btVector3 localInertia(0,0,0);
//
//    if(isDynamic)
//      colShape->calculateLocalInertia(mass, localInertia);
//
//
//
//    startTransform.setOrigin(btVector3(2,10,0));
//
//    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
//    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
//    btRigidBody* body = new btRigidBody(rbInfo);
//
//    m_dynamicsWorld->addRigidBody(body);
}
