#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "btBulletDynamicsCommon.h"


class MultiBody;

class Simulator {
  public:
    Simulator();
    ~Simulator();

    void Initialize();
    void Step();
    

  private:

    void AddWorldObject(MultiBody* _body);


    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btCollisionDispatcher* m_dispatcher;
    btBroadphaseInterface* m_overlappingPairCache;
    btSequentialImpulseConstraintSolver* m_solver;
    btDiscreteDynamicsWorld* m_dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
};

#endif
