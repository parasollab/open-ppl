#ifndef RAPIDCOLLISIONDETECTION_H
#define RAPIDCOLLISIONDETECTION_H

#ifdef USE_RAPID
#include <RAPID.H>

#include "CollisionDetectionMethod.h"

class Rapid: public CollisionDetectionMethod {
 public:
  Rapid();
  virtual ~Rapid();

  /**Using RAPID to check collision between two MultiBodys.
   *Collision is checked in Body level between two MultiBodys,
   *if any of Body from Robot collides with any of Body from obstacle,
   *true will be returned.
   *
   *@note This method doesn't support "Return all info", if 
   *_cdInfo.ret_all_info is true, then error message will be post.
   *@note if RAPID_Collide, the RAPID method, return false, process will 
   *be terminated.
   *@note collision between two ajacent links will be ignore.
   *@return true if Collision found. Otherwise false will be returned.
   *@see Body::GetRapidBody
   */
  virtual bool IsInCollision(shared_ptr<MultiBody> _robot, shared_ptr<MultiBody> _obstacle, 
			     StatClass& _stats, CDInfo& _cdInfo, std::string *_callName=NULL, int _ignoreIAdjacentMultibodies=1);
};
#endif


#endif
