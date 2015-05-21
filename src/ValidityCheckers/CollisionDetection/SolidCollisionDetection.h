#ifndef SOLIDCOLLISIONDETECTION_H
#define SOLIDCOLLISIONDETECTION_H

#ifdef USE_SOLID
#include "CollisionDetectionMethod.h"
#include <SOLID.h>

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

    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);

};
#endif

#endif
