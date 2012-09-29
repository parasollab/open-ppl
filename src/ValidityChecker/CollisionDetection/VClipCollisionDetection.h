#ifndef VCLIPCOLLISIONDETECTION_H
#define VCLIPCOLLISIONDETECTION_H

#ifdef USE_VCLIP
#include "CollisionDetectionMethod.h"
#include <vclip.h>

class Transformation;

class VClip : public CollisionDetectionMethod {
 public:

  VClip();
  virtual ~VClip();

  /**Using VCLIP to check collision between two MultiBodys.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *@note This method doesn't support "Return all info", if 
   *_cdInfo.ret_all_info is true, then error message will be post.
   *@note collision between two ajacent links will be ignore.
   *@return true if Collision found. Otherwise false will be returned.
   *
   *@see Body::GetVclipBody, and GetVclipPose.
   *@see IsInColl_AllInfo_vclip for get all info. 
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
			     StatClass& _stats, CDInfo& _cdInfo, std::string *_callName=NULL, int _ignoreIAdjacentMultibodies=1);
  
  /**Get VclipPose.
   *@todo I don't really know what this is....
   */
  VclipPose GetVClipPose(const Transformation&, const Transformation&);

  
  /**Get all collsion information for given MultiBody.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *More information about collision between two object, such as the closet points between
   *two object, closest distance... all of these information are stored in _cdInfo.
   *
   *@note each obstacle could change the results in _cdInfo
   *Trace back to general IsInCollision call to see how it all
   *gets updated correctly.
   *@see IsInCollision(Environment*, SID, CDInfo& , MultiBody*)
   */
  bool IsInColl_AllInfo_vclip(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
			      CDInfo& _cdInfo, int _ignoreIAdjacentMultibodies=1);
};
#endif

#endif
