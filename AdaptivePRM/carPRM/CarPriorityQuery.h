////////////////////////////////////////////////////
//
//  CarPriorityQuery.h
//
//  derived class of PriorityQuery
//
/////////////////////////////////////////////////////

#ifndef CarPriorityQuery_h
#define CarPriorityQuery_h

#include "PriorityQuery.h"
#include "CarQueryReq.h"

class PriorityLocalPlanners;

class CarPriorityQuery : public PriorityQuery {

public:
  CarPriorityQuery(Input *input, MyQueryCmds *Qinput, CollisionDetection* cd, 
	     DistanceMetric* dm, PriorityLocalPlanners* lp,ConnectMapNodes* cn);

  ~CarPriorityQuery();

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
