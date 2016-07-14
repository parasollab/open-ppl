#ifndef SOLID_COLLISION_DETECTION_H_
#define SOLID_COLLISION_DETECTION_H_

#ifndef NO_SOLID

#include <SOLID.h>

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @ingroup DeadCode
/// @brief TODO DeadCode
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Solid : public CollisionDetectionMethod {
  public:
    Solid();

    virtual void Build(Body* _body);

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);
};
#endif

#endif
