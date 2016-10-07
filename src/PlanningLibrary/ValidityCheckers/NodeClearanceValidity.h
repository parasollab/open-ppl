#ifndef NODECLEARANCEVALIDITY_H
#define NODECLEARANCEVALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class NodeClearanceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;

    NodeClearanceValidity(double _delta = 1.0, string _nfLabel = "");
    NodeClearanceValidity(MPProblemType* _problem, XMLNode& _node);
    virtual ~NodeClearanceValidity();

    virtual void Print(ostream& _os) const;

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);

  private:
    double m_delta;
    string m_nfLabel;
};

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(double _delta, string _nfLabel) :
  ValidityCheckerMethod<MPTraits>(), m_delta(_delta), m_nfLabel(_nfLabel) {
    this->m_name = "NodeClearanceValidity";
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(MPProblemType* _problem, XMLNode& _node):
  ValidityCheckerMethod<MPTraits>(_problem, _node) {
    this->m_name = "NodeClearanceValidity";
    m_delta = _node.Read("delta", true, 1.0, 0.0, MAX_DBL, "Clearance from every other node");
    m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder to be used");
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::~NodeClearanceValidity() {}

template <class MPTraits>
void
NodeClearanceValidity<MPTraits>::Print(ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tdelta::" << m_delta
    << "\tnfLabel::" << m_nfLabel << endl;
}

template <class MPTraits>
bool
NodeClearanceValidity<MPTraits>::IsValidImpl(CfgType& _cfg,
    CDInfo& _cdInfo, const string& _callName) {
  /* TODO: remove ifdef when constness problem in STAPL is fixed*/
#ifndef _PARALLEL
  vector<pair<VID, double> > kClosest;
  this->GetMPProblem()->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(
      this->GetMPProblem()->GetRoadmap(), static_cast<CfgType>(_cfg), back_inserter(kClosest) );

  if(kClosest.empty()) {
    _cfg.SetLabel("VALID", true);
    return true;
  }

  bool result = m_delta < kClosest[0].second;

  _cfg.SetLabel("VALID", result);
  return result;

#else
  stapl_assert(false, "NodeClearanceValidity");
  return false;
#endif
}

#endif
