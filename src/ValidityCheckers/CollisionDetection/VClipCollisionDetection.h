#ifndef VCLIP_COLLISION_DETECTION_H_
#define VCLIP_COLLISION_DETECTION_H_

#ifndef NO_VCLIP

#include "CollisionDetectionMethod.h"

#include <vclip.h>
typedef VclipPose VClipPose;

#include <Transformation.h>
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup CollisionDetection
/// @brief VClip collision detection middleware
///
/// VClip collision detection can be used to compute distances between convex
/// obstacles only, i.e., it can compute clearance for certain types of
/// obstacles.
////////////////////////////////////////////////////////////////////////////////
class VClip : public CollisionDetectionMethod {
  public:
    VClip();

    virtual void Build(Body* _body);

    virtual bool IsInCollision(shared_ptr<Body> _body1,
        shared_ptr<Body> _body2, CDInfo& _cdInfo);

  protected:
    ////////////////////////////////////////////////////////////////////////////
    /// @brief Computes VClip pose information from PMPL transforms
    /// @param _t1 Transform 1
    /// @param _t2 Transform 2
    VClipPose GetVClipPose(const Transformation& _t1, const Transformation& _t2);
};

#endif
#endif
