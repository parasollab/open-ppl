/////////////////////////////////////////////////////////////////////
//
//  NoopCollisionDetection.c
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

#include "NoopCollisionDetection.h"


bool 
NoopCollisionDetection::IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	MultiBody* lineRobot=NULL, bool enablePenetration=true) {
  return false;
}

bool 
NoopCollisionDetection::IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	MultiBody* robot, MultiBody* obstacle) {
  return false;
}

bool 
NoopCollisionDetection::IsInCollision(Environment* env, SID _cdsetid, CDInfo& _cdInfo,
	int robot, int obstacle) {
  return false;
}

