////////////////////////////////////////////////////
//
//  PriorityQuery.h
//
//  derived class of AdaptiveQuery
//
/////////////////////////////////////////////////////

#ifndef PriorityQuery_h
#define PriorityQuery_h

#include "AdaptiveQuery.h"

class PriorityLocalPlanners;

class PriorityQuery : public AdaptiveQuery {

public:
  PriorityQuery(Input *input, MyQueryCmds *Qinput, CollisionDetection* cd, 
	     DistanceMetric* dm, PriorityLocalPlanners* lp,ConnectMapNodes* cn);

  ~PriorityQuery();

  virtual bool PerformQuery(CollisionDetection* cd, ConnectMapNodes* cn, 
			    PriorityLocalPlanners* lp, DistanceMetric* dm);

  virtual bool PerformQuery(Cfg _start, Cfg _goal, CollisionDetection *cd,
			    ConnectMapNodes*cn, PriorityLocalPlanners *lp, DistanceMetric* dm, 
			    SID _lpsid, vector<Cfg>* _path);

  virtual bool GetPathSegment(Cfg _c1, Cfg _c2, CollisionDetection *cd,
			      PriorityLocalPlanners* lp, DistanceMetric * dm,
			      WEIGHT _weight, LPInfo* _ci);
};

#endif
