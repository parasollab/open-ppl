////////////////////////////////////////////////////
//
//  MyQuery.h
//
//  derived class of Query.h
//
/////////////////////////////////////////////////////
#ifndef MyQuery_h
#define MyQuery_h


#include"Query.h"

class MyQuery : public Query {

public:
   MyQuery();
   MyQuery(Input *, QueryCmds*,
        CollisionDetection*, DistanceMetric*, LocalPlanners*,ConnectMapNodes*);
   ~MyQuery();

   virtual bool PerformQuery(CollisionDetection*, ConnectMapNodes*, 
			     LocalPlanners*, DistanceMetric*);
   virtual bool PerformQuery(Cfg _start, Cfg _goal, CollisionDetection*,
        ConnectMapNodes*, LocalPlanners*, DistanceMetric*, SID _lpsid, vector<Cfg>* _path);

};

#endif
