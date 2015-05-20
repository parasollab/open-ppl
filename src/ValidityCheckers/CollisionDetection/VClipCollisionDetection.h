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

    /* Using VCLIP to check collision between two MultiBodys.
     * Collision is checked in Body level between two MultiBodys,
     * if any of Body from Robot collides with any of Body from obstacle,
     * true will be returned.
     *
     * collision between two ajacent links will be ignore.
     */
    virtual bool IsInCollision(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);

  protected:
    VClipPose GetVClipPose(const Transformation&, const Transformation&);

    /* Get all collsion information for given MultiBody.
     * Collision is checked in Body level between two MultiBodys,
     * if any of Body from Robot collides with any of Body from obstacle,
     * true will be returned.
     *
     * More information about collision between two object, such as the closet points between
     * two object, closest distance... all of these information are stored in _cdInfo.
     *
     * each obstacle could change the results in _cdInfo
     * Trace back to general IsInCollision call to see how it all
     * gets updated correctly.
     */
    bool FillCdInfo(shared_ptr<ActiveMultiBody> _robot,
        shared_ptr<MultiBody> _obstacle, CDInfo& _cdInfo,
        size_t _ignoreIAdjacentMultibodies);
};

#endif
#endif
