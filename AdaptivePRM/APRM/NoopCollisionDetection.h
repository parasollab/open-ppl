/////////////////////////////////////////////////////////////////////
//
//  NoopCollisionDetection.h
//
//  General Description
//	A derived class of class CollisionDetection.
//
//	what it does is simple nothing, or 'Noop'. just return false.
//
//  Created
//	07/23/2002 Shawna Thomas 
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
#ifndef NoopCollisionDetection_h
#define NoopCollisionDetection_h


#include "CollisionDetection.h"


class NoopCollisionDetection : public CollisionDetection {

public:

   NoopCollisionDetection() : CollisionDetection() {}
   ~NoopCollisionDetection() {}

   virtual bool IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	MultiBody* lineRobot=NULL, bool enablePenetration=true);

   virtual bool IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	MultiBody* robot, MultiBody* obstacle);

   virtual bool IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	int robot, int obstacle);

};

#endif
