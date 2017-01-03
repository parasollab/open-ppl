#ifndef RAPID_COLLISION_DETECTION_H_
#define RAPID_COLLISION_DETECTION_H_

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// RAPID is typically used to simply and quickly determine collision between
/// two objects, but cannot compute any distance information. The contacts
/// found, i.e., triangle IDs are stored in CDInfo.
////////////////////////////////////////////////////////////////////////////////
class Rapid: public CollisionDetectionMethod {

  public:

    Rapid();

    virtual void Build(Body* _body);

    virtual bool IsInCollision(const Body* const _body1,
        const Body* const _body2, CDInfo& _cdInfo);

};

#endif
