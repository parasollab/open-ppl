////////////////////////////////////////////////////
//
//  CarAdaptiveQuery.h
//
//  derived class of AdaptiveQuery.h
//  specific for car-like robots
//
/////////////////////////////////////////////////////
#ifndef CarAdaptiveQuery_h
#define CarAdaptiveQuery_h


#include "AdaptiveQuery.h"
#include "CarQueryReq.h"


class CarAdaptiveQuery : public AdaptiveQuery {

public:
   CarAdaptiveQuery(Input *, MyQueryCmds*,
        CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
   ~CarAdaptiveQuery() {}

   virtual bool PerformQuery(CollisionDetection*, ConnectMapNodes*, 
   			     LocalPlanners*, DistanceMetric*);
   virtual bool PerformQuery(Cfg _start, Cfg _goal, CollisionDetection*,
        ConnectMapNodes*, LocalPlanners*, DistanceMetric*, SID _lpsid, vector<Cfg>* _path);
   
   virtual bool GetPathSegment(Cfg, Cfg, CollisionDetection*, LocalPlanners*,
        DistanceMetric*, WEIGHT, LPInfo*);
};

#endif
