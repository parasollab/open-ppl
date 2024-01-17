#include "ObstacleClearanceValidity.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

ObstacleClearanceValidity::ObstacleClearanceValidity() {
  this->SetName("ObstacleClearance");
}

ObstacleClearanceValidity::ObstacleClearanceValidity(XMLNode& _node)
    : ValidityCheckerMethod(_node) {
  this->SetName("ObstacleClearance");

  m_clearanceThreshold =
      _node.Read("clearanceThreshold", true, 1.0, -MAX_DBL, MAX_DBL,
                 "Minimum clearance from obstacles for a configuration to be "
                 "considered valid");

  m_vcLabel = _node.Read("vcLabel", true, "",
                         "The underlying validity checker "
                         "label (must be a PQP Solid type).");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void ObstacleClearanceValidity::Initialize() {
  // Ensure that the named validity checker is a PQP solid collision detector.
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto cd = dynamic_cast<CollisionDetectionValidity*>(vc);
  if (!cd)
    throw ParseException(WHERE)
        << "Named validity checker '" << m_vcLabel
        << "' for ObstacleClearanceValidity '" << this->GetLabel()
        << "' is not a CollisionDetectionValidity object.";

  PQPSolid* pqp = dynamic_cast<PQPSolid*>(cd->GetCDMethod());
  if (!pqp)
    throw ParseException(WHERE)
        << "Named validity checker '" << m_vcLabel
        << "' for ObstacleClearanceValidity '" << this->GetLabel()
        << "' is not a PQPSolid type.";
}

void ObstacleClearanceValidity::Print(std::ostream& _os) const {
  ValidityCheckerMethod::Print(_os);
  _os << "\tRequired Clearance: " << m_clearanceThreshold
      << "\n\tValidity Checker: " << m_vcLabel << std::endl;
}

/*------------------- ValidityCheckerMethod Overrides ------------------------*/

bool ObstacleClearanceValidity::IsValidImpl(Cfg& _cfg,
                                            CDInfo& _cdInfo,
                                            const std::string& _callName) {
  // Clear out the CD info and make sure we request everything to get the
  // clearance info.
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  const bool valid = vc->IsValid(_cfg, _cdInfo, _callName);

  // If the cfg is clear but not outside the threshold, mark it invalid.
  if (valid && _cdInfo.m_minDist < m_clearanceThreshold) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  // Otherwise the validity is already correctly determined.
  return valid;
}

/*----------------------------------------------------------------------------*/
