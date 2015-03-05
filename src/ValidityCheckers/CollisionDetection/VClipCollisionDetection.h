#ifndef VCLIPCOLLISIONDETECTION_H_
#define VCLIPCOLLISIONDETECTION_H_

#ifdef USE_VCLIP

#include "CollisionDetectionMethod.h"

#include <vclip.h>
typedef VclipPose VClipPose; //typedef allows our naming convention on VClip objects.

#include <Transformation.h>
using namespace mathtool;

class VClip : public CollisionDetectionMethod {
  public:
    VClip();
    virtual ~VClip();

    /* Using VCLIP to check collision between two MultiBodys.
     * Collision is checked in Body level between two MultiBodys,
     * if any of Body from Robot collides with any of Body from obstacle,
     * true will be returned.
     *
     * collision between two ajacent links will be ignore.
     */
    virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        StatClass& _stats, CDInfo& _cdInfo, const string& _callName, int _ignoreIAdjacentMultibodies = 1);

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
    bool FillCdInfo(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle,
        CDInfo& _cdInfo, int _ignoreIAdjacentMultibodies=1);
};

#endif
#endif
