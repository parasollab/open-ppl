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

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
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

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};

#endif
