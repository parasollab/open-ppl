#ifndef MEDIALAXISCLEARANCEVALIDITY_H_
#define MEDIALAXISCLEARANCEVALIDITY_H_

#include "ValidityCheckerMethod.h"
#include "Utilities/MedialAxisUtilities.h"
#include "ValidityCheckers/CollisionDetectionValidity.h"

template<class MPTraits>
class MedialAxisClearanceValidity : public ValidityCheckerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MedialAxisClearanceValidity(const MedialAxisUtility<MPTraits>& _m = MedialAxisUtility<MPTraits>(), double _c = 0.001);
    MedialAxisClearanceValidity(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MedialAxisClearanceValidity() {}

    void ParseXML(XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;

    virtual bool IsInsideObstacle(const CfgType& _cfg);

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);

    vector< pair<CfgType,CfgType> >& GetHistory();
    void ClearHistory();

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_clearance;
    vector< pair<CfgType,CfgType> > m_history;
};

template<class MPTraits>
MedialAxisClearanceValidity<MPTraits>::MedialAxisClearanceValidity(const MedialAxisUtility<MPTraits>& _m, double _c):
  m_medialAxisUtility(_m), m_clearance(_c){
    this->m_name = "MedialAxisClearance";
  }

template<class MPTraits>
MedialAxisClearanceValidity<MPTraits>::MedialAxisClearanceValidity(MPProblemType* _problem, XMLNodeReader& _node) :
  ValidityCheckerMethod<MPTraits>(_problem, _node), m_medialAxisUtility(_problem, _node){
    this->m_name = "MedialAxisClearance";
    ParseXML(_node);
    m_history.clear();
  }

template<class MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::ParseXML(XMLNodeReader& _node){
  this->m_clearance  = _node.numberXMLParameter("maClearance", true, 0.1, 0.0, MAX_DBL, "Medial Axis Validity Clearance");
  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::PrintOptions(ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::PrintOptions(_os);
  _os << "\tMaximum distance from medial axis::" << m_clearance << endl;
  _os << "\tMedialAxisUtility::" << endl;
  m_medialAxisUtility.PrintOptions(_os);
}


template<class MPTraits>
bool
MedialAxisClearanceValidity<MPTraits>::IsInsideObstacle(const CfgType& _cfg){
  typename MPProblemType::ValidityCheckerPointer vcm = this->GetMPProblem()->
    GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());
  return vcm->IsInsideObstacle(_cfg);
}

template<class MPTraits>
bool
MedialAxisClearanceValidity<MPTraits>::IsValidImpl(CfgType& _cfg,
    CDInfo& _cdInfo, const string& _callName) {
  Environment* env = this->GetMPProblem()->GetEnvironment();

  typename MPProblemType::ValidityCheckerPointer vc = this->GetMPProblem()->
    GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());
  bool isFree = vc->IsValid(_cfg, _cdInfo, _callName);

  if(!isFree){
    _cfg.SetLabel("VALID", !isFree);
    return !isFree;
  }

  CfgType origCfg = _cfg;
  CfgType tmpCfg = _cfg;
  if(!m_medialAxisUtility.PushToMedialAxis(tmpCfg, env->GetBoundary())) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  m_history.push_back(make_pair(origCfg, tmpCfg));

  string dmLabel = m_medialAxisUtility.GetDistanceMetricLabel();
  double dist = this->GetMPProblem()->GetDistanceMetric(dmLabel)->Distance(tmpCfg, _cfg);
  bool result = dist < m_clearance;

  _cfg.SetLabel("VALID", result);

  return result;
}

template<class MPTraits>
vector< pair<typename MPTraits::CfgType, typename MPTraits::CfgType> >&
MedialAxisClearanceValidity<MPTraits>::GetHistory() {
  return m_history;
}

template<class MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::ClearHistory() {
  m_history.clear();
}

#endif
