/////////////////////////////////////////////////////////////////////
//
//  NoopLocalPlanners.h
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
#ifndef NoopLocalPlanners_h
#define NoopLocalPlanners_h


#include "LocalPlanners.h"


class NoopLocalPlanners : public LocalPlanners {

public:

   NoopLocalPlanners() : LocalPlanners() {}
   ~NoopLocalPlanners() {}

   // base structure (function) for both straightline & rotate_at_s local planners.
   virtual bool IsConnected_straightline_simple(Environment *env,
	CollisionDetection *, DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  
    
   virtual bool IsConnected_astar(Environment *env,
        CollisionDetection *, DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
   

   /////////////////////////////////////////////////////////////////////
   // data 
   /////////////////////////////////////////////////////////////////////

};

#endif
