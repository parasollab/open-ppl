#ifndef MEDIAL_AXIS_CLEARANCE_VALIDITY_H_
#define MEDIAL_AXIS_CLEARANCE_VALIDITY_H_

#include "ValidityCheckerMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MedialAxisClearanceValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType CfgType;

    MedialAxisClearanceValidity(const MedialAxisUtility<MPTraits>& _m = MedialAxisUtility<MPTraits>(), double _c = 0.001);
    MedialAxisClearanceValidity(XMLNode& _node);
    virtual ~MedialAxisClearanceValidity() {}

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName);

    vector< pair<CfgType,CfgType> >& GetHistory();
    void ClearHistory();

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_clearance;
    vector< pair<CfgType,CfgType> > m_history;
};

template <typename MPTraits>
MedialAxisClearanceValidity<MPTraits>::
MedialAxisClearanceValidity(const MedialAxisUtility<MPTraits>& _m, double _c) :
  m_medialAxisUtility(_m), m_clearance(_c) {
    this->SetName("MedialAxisClearance");
  }

template <typename MPTraits>
MedialAxisClearanceValidity<MPTraits>::
MedialAxisClearanceValidity(XMLNode& _node) :
  ValidityCheckerMethod<MPTraits>(_node),
  m_medialAxisUtility(_node) {
    this->SetName("MedialAxisClearance");
    ParseXML(_node);
    m_history.clear();
  }

template <typename MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::
ParseXML(XMLNode& _node) {
  this->m_clearance  = _node.Read("maClearance", true, 0.1, 0.0, MAX_DBL,
      "Medial Axis Validity Clearance");
}

template <typename MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::
Print(ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tMaximum distance from medial axis::" << m_clearance << endl;
  _os << "\tMedialAxisUtility::" << endl;
  m_medialAxisUtility.Print(_os);
}


template <typename MPTraits>
bool
MedialAxisClearanceValidity<MPTraits>::
IsValidImpl(CfgType& _cfg,
    CDInfo& _cdInfo, const string& _callName) {
  Environment* env = this->GetEnvironment();

  auto vc = this->GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());
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
  double dist = this->GetDistanceMetric(dmLabel)->Distance(tmpCfg, _cfg);
  bool result = dist < m_clearance;

  _cfg.SetLabel("VALID", result);

  return result;
}

template <typename MPTraits>
vector< pair<typename MPTraits::CfgType, typename MPTraits::CfgType> >&
MedialAxisClearanceValidity<MPTraits>::
GetHistory() {
  return m_history;
}

template <typename MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::
ClearHistory() {
  m_history.clear();
}

#endif
