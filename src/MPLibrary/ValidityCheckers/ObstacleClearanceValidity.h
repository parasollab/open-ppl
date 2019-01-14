#ifndef PMPL_OBSTACLE_CLEARANCE_VALIDITY_H_
#define PMPL_OBSTACLE_CLEARANCE_VALIDITY_H_

#include "ValidityCheckerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Marks configurations as valid iff they are at least some minimum threshold
/// distance away from the nearest obstacle.
///
/// @todo Re-implement this as a derived class of CollisionDetectionValidity and
///       remove the 'GetCDMethod' function.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ObstacleClearanceValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    ObstacleClearanceValidity();

    ObstacleClearanceValidity(XMLNode& _node);

    virtual ~ObstacleClearanceValidity() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_clearanceThreshold; ///< The minimum clearance threshold.
    std::string m_vcLabel;       ///< The VC label, must point to pqp solid.

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleClearanceValidity<MPTraits>::
ObstacleClearanceValidity() {
  this->SetName("ObstacleClearance");
}


template <typename MPTraits>
ObstacleClearanceValidity<MPTraits>::
ObstacleClearanceValidity(XMLNode& _node) :
    ValidityCheckerMethod<MPTraits>(_node) {
  this->SetName("ObstacleClearance");

  m_clearanceThreshold = _node.Read("clearanceThreshold", true, 1.0, -MAX_DBL,
      MAX_DBL, "Minimum clearance from obstacles for a configuration to be "
      "considered valid");

  m_vcLabel = _node.Read("vcLabel", true, "", "The underlying validity checker "
      "label (must be a PQP Solid type).");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ObstacleClearanceValidity<MPTraits>::
Initialize() {
  // Ensure that the named validity checker is a PQP solid collision detector.
  auto vc = this->GetValidityChecker(m_vcLabel);

  CollisionDetectionValidity<MPTraits>* cd =
      dynamic_cast<CollisionDetectionValidity<MPTraits>*>(vc.get());
  if(!cd)
    throw ParseException(WHERE) << "Named validity checker '" << m_vcLabel
                                << "' for ObstacleClearanceValidity '"
                                << this->GetLabel()
                                << "' is not a CollisionDetectionValidity object.";

  PQPSolid* pqp = dynamic_cast<PQPSolid*>(cd->GetCDMethod());
  if(!pqp)
    throw ParseException(WHERE) << "Named validity checker '" << m_vcLabel
                                << "' for ObstacleClearanceValidity '"
                                << this->GetLabel()
                                << "' is not a PQPSolid type.";
}


template <typename MPTraits>
void
ObstacleClearanceValidity<MPTraits>::
Print(std::ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tRequired Clearance: " << m_clearanceThreshold
      << "\n\tValidity Checker: " << m_vcLabel
      << std::endl;
}

/*------------------- ValidityCheckerMethod Overrides ------------------------*/

template <typename MPTraits>
bool
ObstacleClearanceValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  // Clear out the CD info and make sure we request everything to get the
  // clearance info.
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  auto vc = this->GetValidityChecker(m_vcLabel);
  const bool valid = vc->IsValid(_cfg, _cdInfo, _callName);

  // If the cfg is clear but not outside the threshold, mark it invalid.
  if(valid && _cdInfo.m_minDist < m_clearanceThreshold) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  // Otherwise the validity is already correctly determined.
  return valid;
}

/*----------------------------------------------------------------------------*/

#endif
