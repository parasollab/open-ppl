/////////////////////////////////////////////////////////////////////
//
// PriorityLocalPlanners.h
//
//  General Description
//	A derived class of class LocalPlanners. Edge weights are 
//      the probability of being connected. Always returns true.
//
//  Created
//	07/24/2002 Shawna Thomas 
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
#ifndef PriorityLocalPlanners_h
#define PriorityLocalPlanners_h


#include "LocalPlanners.h"


class PriorityLocalPlanners : public LocalPlanners {

public:

   PriorityLocalPlanners() : LocalPlanners() {
     lambda = 1;
   }
   PriorityLocalPlanners(double _lambda) : LocalPlanners() {
     lambda = _lambda;
   }
   ~PriorityLocalPlanners() {}

   double CalWeight(Environment* env, Cfg& _c1, Cfg& _c2, DistanceMetric* dm, LPInfo *info);
   
   virtual bool IsConnected_straightline_simple(Environment *env, CollisionDetection *,
						DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, 
						LPInfo *info);
   
   virtual bool IsConnected_rotate_at_s(Environment *env,CollisionDetection *,
					DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, 
					LPInfo *info);

   virtual bool IsConnected_astar(Environment *env,CollisionDetection *,
				  DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, 
				  LPInfo *info);
 
   inline double& Lambda() { return lambda; }

protected:
   ///////////////////////////////
   // Data
   ///////////////////////////////
   double lambda;

};

#endif
