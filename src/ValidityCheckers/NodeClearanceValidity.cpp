#include "NodeClearanceValidity.h"
#include "DistanceMetrics.h"
#include "NeighborhoodFinder.h"


NodeClearanceValidity::NodeClearanceValidity() : ValidityCheckerMethod() {
  m_name = "NodeClearanceValidity";
}

NodeClearanceValidity::NodeClearanceValidity(double _delta, string _dmLabel, string _nfLabel) : 
    ValidityCheckerMethod(), m_delta(_delta), m_dmLabel(_dmLabel), m_nfLabel(_nfLabel) {
  m_name = "NodeClearanceValidity";
}

NodeClearanceValidity::NodeClearanceValidity (XMLNodeReader& _node, MPProblem* _problem) :
    ValidityCheckerMethod(_node, _problem) {
  m_name = "NodeClearanceValidity";
  m_delta = _node.numberXMLParameter("delta", true, 1.0, 0.0, 1.0, "Clearance from every other node");
  m_dmLabel = _node.stringXMLParameter("dmMethod",true,"","Distance metric to be used");     
  m_nfLabel = _node.stringXMLParameter("nfMethod",true,"","Neighborhood Finder to be used");    
}

NodeClearanceValidity::~NodeClearanceValidity() {}

bool NodeClearanceValidity::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string* _callName) {
  /* remove ifdef when constness problem in STAPL is fixed*/
  #ifndef _PARALLEL
  typedef RoadmapGraph<CfgType, WeightType>::VID VID; 
  vector<VID> KClosest;

  GetMPProblem()->GetNeighborhoodFinder()->GetMethod(m_nfLabel)->KClosest(
      GetMPProblem()->GetRoadmap(), static_cast<CfgType>(_cfg), 1, back_inserter(KClosest) );

  if(KClosest.empty()) {
    _cfg.SetLabel("VALID", true);
    return true;
  }

  DistanceMetric::DistanceMetricPointer dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);
  CfgType nearest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(KClosest[0])->property();
  bool result = m_delta < dm->Distance(_env, nearest, _cfg);
  
  _cfg.SetLabel("VALID", result);
  return result;
  
  #else
  stapl_assert(false,"NodeClearanceValidity");
  #endif
}

