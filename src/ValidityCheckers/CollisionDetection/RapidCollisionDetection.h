#ifndef RAPID_COLLISION_DETECTION_H_
#define RAPID_COLLISION_DETECTION_H_

#ifndef NO_RAPID

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief RAPID collision detection middleware
///
/// RAPID is typically used to simply and quickly determine collision between
/// two objects, but cannot compute any distance information. The contacts
/// found, i.e., triangle IDs are stored in CDInfo.
////////////////////////////////////////////////////////////////////////////////
class Rapid: public CollisionDetectionMethod {

  public:

    Rapid();

    virtual void Build(Body* _body);

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};

#endif
#endif
