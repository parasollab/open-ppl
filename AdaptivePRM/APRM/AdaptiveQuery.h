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

   typedef pair<Cfg, Cfg> CfgPairType;

   typedef pair<CfgPairType, double> DIS_TYPE;
   /*
   bool DIST_Compare(const DIST_TYPE& _cc1, const DIST_TYPE& _cc2) {
     return (_cc1.second < _cc2.second );
   }
*/

   virtual void FindKClosestPairs(vector<CfgPairType>& kp, Environment* _env, DistanceMetric* dm,
				  const int kclosest, const Cfg& c, const vector<Cfg>& vertices, 
				  SID dmsid);

   /////////////////////////////////////////////////////////
   //  data 
   /////////////////////////////////////////////////////////
   QueryRequirementsObject queryReq;
   int checkAllNodes;
};

#endif
