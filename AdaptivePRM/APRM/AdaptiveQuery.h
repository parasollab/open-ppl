////////////////////////////////////////////////////
//
//  AdaptiveQuery.h
//
//  derived class of Query.h
//
/////////////////////////////////////////////////////
#ifndef AdaptiveQuery_h
#define AdaptiveQuery_h


#include"Query.h"
#include"QueryRequirements.h"
#include"MyQueryCmds.h"

class AdaptiveQuery : public Query {

public:
   //AdaptiveQuery();
   AdaptiveQuery(Input *, MyQueryCmds*, CollisionDetection*, DistanceMetric*, 
		 LocalPlanners*,ConnectMapNodes*);
   ~AdaptiveQuery();

   virtual bool PerformQuery(CollisionDetection*, ConnectMapNodes*, 
			     LocalPlanners*, DistanceMetric*);
   virtual bool PerformQuery(Cfg _start, Cfg _goal, CollisionDetection*,
			     ConnectMapNodes*, LocalPlanners*, DistanceMetric*, 
			     SID _lpsid, vector<Cfg>* _path);

   virtual bool GetPathSegment(Cfg, Cfg, CollisionDetection*, LocalPlanners*,
			       DistanceMetric*, WEIGHT, LPInfo*);

   virtual bool removeBadNodes(vector <Cfg>& cfgs, Environment *env,
			       CollisionDetection *cd, SID cdsetid);

   /////////////////////////////////////////////////////////
   //  data 
   /////////////////////////////////////////////////////////
   QueryRequirementsObject queryReq;
   int checkAllNodes;

};

#endif
