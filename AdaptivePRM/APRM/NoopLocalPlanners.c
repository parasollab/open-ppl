/////////////////////////////////////////////////////////////////////
//
//  NoopLocalPlanners.c
//
//  General Description
//	A derived class of class LocalPlanners.
//
//	what it does is simple nothing, or 'Noop'. just return true.
//     
//  Created
//	02/09/2001 Guang Song 
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////

#include "NoopLocalPlanners.h"


bool 
NoopLocalPlanners::IsConnected_straightline_simple(Environment *env,
	CollisionDetection *,DistanceMetric *, Cfg& _c1, Cfg& _c2, 
	LP& _lp, LPInfo *info) {
  return true;
}
  
bool
NoopLocalPlanners::IsConnected_astar(Environment *env,
	CollisionDetection *,DistanceMetric *, Cfg& _c1, Cfg& _c2,
	LP& _lp, LPInfo *info) {
  return true;
}
