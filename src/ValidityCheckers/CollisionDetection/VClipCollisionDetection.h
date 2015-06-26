#ifndef VCLIPCOLLISIONDETECTION_H_
#define VCLIPCOLLISIONDETECTION_H_

#ifdef USE_VCLIP

#include "CollisionDetectionMethod.h"

#include <vclip.h>
typedef VclipPose VClipPose; //typedef allows our naming convention on VClip objects.

#include <Transformation.h>
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class VClip : public CollisionDetectionMethod {
  public:
    VClip();

    virtual void Build(Body* _body);

    /* Using VCLIP to check collision between two MultiBodys.
     * Collision is checked in Body level between two MultiBodys,
     * if any of Body from Robot collides with any of Body from obstacle,
     * true will be returned.
     *
     * collision between two ajacent links will be ignore.
     */
    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);

  protected:
    VClipPose GetVClipPose(const Transformation&, const Transformation&);
};

#endif
#endif
