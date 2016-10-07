#ifndef SPHERES_COLLISION_DETECTION_H_
#define SPHERES_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Collision detection using bounding spheres only.
///
/// Collision detection using bounding spheres. This implies that collisions
/// will be unsure, but no collision is certain.
////////////////////////////////////////////////////////////////////////////////
class BoundingSpheres : public CollisionDetectionMethod {

  public:

    BoundingSpheres();

    virtual void Build(Body* _body) {}

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief Collision detection using inscribed spheres only.
///
/// Collision detection using inscribed spheres only. This implies that
/// collisions will be sure, but no collision is uncertain.
////////////////////////////////////////////////////////////////////////////////
class InsideSpheres : public CollisionDetectionMethod {

  public:

    InsideSpheres();

    virtual void Build(Body* _body) {}

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};

#endif
