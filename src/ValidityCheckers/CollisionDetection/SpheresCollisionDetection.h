#ifndef SPHERESCOLLISIONDETECTION_H
#define SPHERESCOLLISIONDETECTION_H

#include "CollisionDetectionMethod.h"

class BoundingSpheres : public CollisionDetectionMethod {
  public:
    BoundingSpheres();
    virtual ~BoundingSpheres();

    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies=1);
};


class InsideSpheres : public CollisionDetectionMethod {
  public:
    InsideSpheres();
    virtual ~InsideSpheres();

    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies=1);
};

#endif
