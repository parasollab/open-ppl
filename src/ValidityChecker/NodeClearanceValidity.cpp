#include "NodeClearanceValidity.h"
#include "DistanceMetrics.h"
#include "MPProblem/MPRegion.h"
#include "NeighborhoodFinder.h"

using namespace std;

NodeClearanceValidity::NodeClearanceValidity(double _delta, string _dmLabel, string _nfLabel) : 
  m_delta(_delta), m_dmLabel(_dmLabel), m_nfLabel(_nfLabel) {
  }

NodeClearanceValidity::NodeClearanceValidity (XMLNodeReader& _node, MPProblem* _problem) :
  ValidityCheckerMethod(_node, _problem) {
    m_delta = _node.numberXMLParameter("delta", true, 1.0, 0.0, 1.0, "Clearance from every other node");
    m_dmLabel = _node.stringXMLParameter("dmMethod",true,"","Distance metric to be used");     
    m_nfLabel = _node.stringXMLParameter("nfMethod",true,"","Neighborhood Finder to be used");    
  }

bool NodeClearanceValidity::IsValid(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, bool _enablePenetration, string* _callName){
  /* remove ifdef when constness problem in STAPL is fixed*/
  #ifndef _PARALLEL
  typedef RoadmapGraph<CfgType, WeightType>::VID VID; 
  
  DistanceMetric::DistanceMetricPointer dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmLabel);
  vector<VID> KClosest;

  GetMPProblem()->GetNeighborhoodFinder()->GetMethod(m_nfLabel)->KClosest(
      GetMPProblem()->GetMPRegion(0)->GetRoadmap(), static_cast<CfgType>(_cfg) , 1, back_inserter(KClosest) );

  if ( KClosest.empty() ){
    _cfg.SetLabel("VALID", true);
    return true;
  }

  CfgType nearest = GetMPProblem()->GetMPRegion(0)->GetRoadmap()->m_pRoadmap->find_vertex(KClosest[0])->property();

  double dist = dm->Distance(_env, nearest, _cfg);
  bool result = m_delta < dist; 
  _cfg.SetLabel("VALID", result);
  return result;
  #else
  stapl_assert(false,"NodeClearanceValidity");
  #endif
}
