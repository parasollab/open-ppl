#ifndef _QUERY_EVALUATION_H
#define _QUERY_EVALUATION_H

#include "Query.h"

template <class CFG, class WEIGHT>
class QueryEvaluation : public MapEvaluationMethod<CFG,WEIGHT> {

  public:

  QueryEvaluation(char * queryFileName, 
                 Stat_Class& Stats, CollisionDetection* cd , 
                 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
                 LocalPlanners<CFG,WEIGHT>* lp);
  QueryEvaluation(CFG _start, CFG _goal,
                 Stat_Class& Stats, CollisionDetection* cd , 
                 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
                 LocalPlanners<CFG,WEIGHT>* lp);
  ~QueryEvaluation();

  virtual bool evaluate(Roadmap<CFG,WEIGHT>* rmap);


  ///////////////////////////
  //
  //  DATA MEMBERS
  //
  ///////////////////////////

  private:

  Query<CFG, WEIGHT> query;
  Stat_Class mystats;
  CollisionDetection* mycd;
  ConnectMap<CFG, WEIGHT> *mycm;
  DistanceMetric * mydm;
  LocalPlanners<CFG,WEIGHT>* mylp;


};  


template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
QueryEvaluation(char * queryFileName, 
                 Stat_Class& Stats, CollisionDetection* cd , 
                 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
                 LocalPlanners<CFG,WEIGHT>* lp) : query(queryFileName) , mycd(cd) , mycm(cm) , mydm(dm) , mylp(lp) {

}


template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
QueryEvaluation(CFG _start, CFG _goal,
                 Stat_Class& Stats, CollisionDetection* cd , 
                 ConnectMap<CFG, WEIGHT> *cm, DistanceMetric * dm,
                 LocalPlanners<CFG,WEIGHT>* lp) : query(_start, _goal) , mycd(cd) , mycm(cm) , mydm(dm) , mylp(lp) {
}


template <class CFG, class WEIGHT>
QueryEvaluation<CFG, WEIGHT>::
~QueryEvaluation(){}


template <class CFG, class WEIGHT>
bool
QueryEvaluation<CFG, WEIGHT>::
evaluate(Roadmap<CFG,WEIGHT>* rmap) {

  bool queryResult;
  int oriVertID = rmap->m_pRoadmap->getVertIDs();

  queryResult = query.PerformQuery(rmap, mystats,mycd,mycm,mylp,mydm);
  
  //remove nodes in query.query from rmap
  for(int i = 0; i< query.query.size(); i++){
    rmap->m_pRoadmap->DeleteVertex(query.query[i]);  
  }

  rmap->m_pRoadmap->setVertIDs(oriVertID);

  return queryResult;

}

#endif
