// $Id$
/////////////////////////////////////////////////////////////////////
//
//  DynamicsLocalPlanners.h
//
//  General Description
//	A derived class of class LocalPlanners. Mainly to be used
//      in protein folding, molecular docking as other computational
//	biology stuff(and others which need correct dynamics.) 
//
//	The correct dynamics is dispensable for good simulation and 
//	it can be coded somehow into LocalPlanners.
//
//
//  Created
//	06/10/2000 Guang Song 
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////
#ifndef DynamicsLocalPlanners_h
#define DynamicsLocalPlanners_h


#include "LocalPlanners.h"

class DynamicsLocalPlanners : public LocalPlanners {

public:

   DynamicsLocalPlanners() : LocalPlanners() {}
   ~DynamicsLocalPlanners() {}

   // base structure (function) for both straightline & rotate_at_s local planners.
   // detail dynamics can be coded here.
   virtual bool IsConnected_straightline_simple(Environment *env,
	CollisionDetection *,DistanceMetric *, Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
  
   // to be implemented later.
   //virtual bool IsConnected_astar(Environment *env,CollisionDetection *,DistanceMetric *,
   //                             Cfg& _c1, Cfg& _c2, LP& _lp, LPInfo *info);
   
};

#endif
