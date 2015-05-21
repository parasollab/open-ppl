#ifndef RAPIDCOLLISIONDETECTION_H_
#define RAPIDCOLLISIONDETECTION_H_

#ifdef USE_RAPID

#include "CollisionDetectionMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class Rapid: public CollisionDetectionMethod {
  public:
    Rapid();

    virtual void Build(Body* _body);

    /* Using RAPID to check collision between two MultiBodys.
     * Collision is checked in Body level between two MultiBodys,
     * if any of Body from Robot collides with any of Body from obstacle,
     * true will be returned.
     *
     * This method doesn't support "Return all info", if
     * _cdInfo.ret_all_info is true, then error message will be post.
     *
     * if RAPID_Collide, the RAPID method, return false, process will
     * be terminated.
     *
     * collision between two ajacent links will be ignored.
     */
    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);
};

#endif
#endif
