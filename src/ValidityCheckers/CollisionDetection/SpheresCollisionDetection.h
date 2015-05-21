#ifndef SPHERES_COLLISION_DETECTION_H_
#define SPHERES_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BoundingSpheres : public CollisionDetectionMethod {
  public:
    BoundingSpheres();

    virtual void Build(Body* _body) {};

    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class InsideSpheres : public CollisionDetectionMethod {
  public:
    InsideSpheres();

    virtual void Build(Body* _body) {};

    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);
};

#endif
