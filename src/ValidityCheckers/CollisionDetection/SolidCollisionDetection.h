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
    virtual ~Solid();

    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies=1);

};
#endif

#endif
