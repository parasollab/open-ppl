#ifndef NODECLEARANCEVALIDITY_H
#define NODECLEARANCEVALIDITY_H

#include "ValidityCheckerMethod.h"

template<class MPTraits>
class NodeClearanceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    NodeClearanceValidity(double _delta = 1.0, string _dmLabel = "", string _nfLabel = "");
    NodeClearanceValidity(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~NodeClearanceValidity();

    virtual void PrintOptions(ostream& _os);
    
    virtual bool IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
        CDInfo& _cdInfo, string* _callName);

  private:
    double m_delta;
    string m_dmLabel;
    string m_nfLabel;
};

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(double _delta, string _dmLabel, string _nfLabel) : 
  ValidityCheckerMethod<MPTraits>(), m_delta(_delta), m_dmLabel(_dmLabel), m_nfLabel(_nfLabel) {
    this->m_name = "NodeClearanceValidity";
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(MPProblemType* _problem, XMLNodeReader& _node):
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    this->m_name = "NodeClearanceValidity";
    m_delta = _node.numberXMLParameter("delta", true, 1.0, 0.0, MAX_DBL, "Clearance from every other node");
    m_dmLabel = _node.stringXMLParameter("dmLabel", true, "", "Distance metric to be used");     
    m_nfLabel = _node.stringXMLParameter("nfLabel", true, "", "Neighborhood Finder to be used"); 
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::~NodeClearanceValidity() {}

template <class MPTraits>
void
NodeClearanceValidity<MPTraits>::PrintOptions(ostream& _os){
  ValidityCheckerMethod<MPTraits>::PrintOptions(_os);
  _os << "\tdelta::" << m_delta
    << "\tdmLabel::" << m_dmLabel
    << "\tnfLabel::" << m_nfLabel << endl;
}

template <class MPTraits>
bool
NodeClearanceValidity<MPTraits>::IsValidImpl(Cfg& _cfg, Environment* _env, StatClass& _stats, 
    CDInfo& _cdInfo, string* _callName) {
  /* TODO: remove ifdef when constness problem in STAPL is fixed*/
#ifndef _PARALLEL
  vector<VID> KClosest;
  this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel)->KClosest(
      this->GetMPProblem()->GetRoadmap(), static_cast<CfgType>(_cfg), 1, back_inserter(KClosest) );

  if(KClosest.empty()) {
    _cfg.SetLabel("VALID", true);
    return true;
  }

  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->find_vertex(KClosest[0])->property();
  bool result = (m_delta < dm->Distance(_env, nearest, _cfg));

  _cfg.SetLabel("VALID", result);
  return result;

#else
  stapl_assert(false, "NodeClearanceValidity");
  return false;
#endif
}

#endif
