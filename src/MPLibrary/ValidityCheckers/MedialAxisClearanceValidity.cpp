#include "MedialAxisClearanceValidity.h"
#include "MPLibrary/MPLibrary.h"

/*----------------------------------------------------------------------------*/


MedialAxisClearanceValidity::
MedialAxisClearanceValidity(const MedialAxisUtility& _m, double _c)
    : m_medialAxisUtility(_m), m_clearance(_c) {
  this->SetName("MedialAxisClearance");
}


MedialAxisClearanceValidity::
MedialAxisClearanceValidity(XMLNode& _node)
    : ValidityCheckerMethod(_node),
      m_medialAxisUtility(_node) {
  this->SetName("MedialAxisClearance");

  this->m_clearance = _node.Read("maClearance", true, 0.1,
      0., std::numeric_limits<double>::max(),
      "Medial Axis Validity Clearance");

  m_history.clear();
}

/*----------------------------------------------------------------------------*/


void
MedialAxisClearanceValidity::
Print(std::ostream& _os) const {
  ValidityCheckerMethod::Print(_os);
  _os << "\tMaximum distance from medial axis: " << m_clearance
      << "\tMedialAxisUtility: "
      << std::endl;
  m_medialAxisUtility.Print(_os);
}


/*----------------------------------------------------------------------------*/


bool
MedialAxisClearanceValidity::
IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  Environment* env = this->GetEnvironment();

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());
  bool isFree = vc->IsValid(_cfg, _cdInfo, _callName);

  if(!isFree){
    _cfg.SetLabel("VALID", !isFree);
    return !isFree;
  }

  Cfg origCfg = _cfg;
  Cfg tmpCfg = _cfg;
  if(!m_medialAxisUtility.PushToMedialAxis(tmpCfg, env->GetBoundary())) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  m_history.push_back(make_pair(origCfg, tmpCfg));

  const std::string dmLabel = m_medialAxisUtility.GetDistanceMetricLabel();
  const double dist = this->GetMPLibrary()->GetDistanceMetric(dmLabel)->Distance(_cfg, tmpCfg);
  const bool result = dist < m_clearance;

  _cfg.SetLabel("VALID", result);

  return result;
}

/*----------------------------------------------------------------------------*/


std::vector< std::pair<Cfg, Cfg> >&
MedialAxisClearanceValidity::
GetHistory() {
  return m_history;
}



void
MedialAxisClearanceValidity::
ClearHistory() {
  m_history.clear();
}

/*----------------------------------------------------------------------------*/
