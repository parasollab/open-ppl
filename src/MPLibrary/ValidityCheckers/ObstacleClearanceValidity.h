#ifndef OBSTACLE_CLEARANCE_VALIDITY_H_
#define OBSTACLE_CLEARANCE_VALIDITY_H_

#include "ValidityCheckerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// TODO
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

    ObstacleClearanceValidity(double _obstClearance = 1.0,
        const ClearanceUtility<MPTraits>& _c = ClearanceUtility<MPTraits>());

    ObstacleClearanceValidity(XMLNode& _node);

    virtual ~ObstacleClearanceValidity() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const string& _callName) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_obstClearance;
    ClearanceUtility<MPTraits> m_clearanceUtility;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ObstacleClearanceValidity<MPTraits>::
ObstacleClearanceValidity(double _obstClearance,
    const ClearanceUtility<MPTraits>& _c) :
    m_obstClearance(_obstClearance), m_clearanceUtility(_c) {
  this->SetName("ObstacleClearance");
}


template <typename MPTraits>
ObstacleClearanceValidity<MPTraits>::
ObstacleClearanceValidity(XMLNode& _node) :
    ValidityCheckerMethod<MPTraits>(_node), m_clearanceUtility(_node) {
  this->SetName("ObstacleClearance");
  m_obstClearance = _node.Read("obstClearance", true, 1.0, -MAX_DBL, MAX_DBL,
      "Required clearance from obstacles");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
ObstacleClearanceValidity<MPTraits>::
Print(ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tRequired Clearance::" << m_obstClearance << endl;
  _os << "\tClearanceUtil::" << endl;
  m_clearanceUtility.Print(_os);
}

/*------------------- ValidityCheckerMethod Overrides ------------------------*/

template <typename MPTraits>
bool
ObstacleClearanceValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const string& _callName) {
  Environment* env = this->GetEnvironment();

  auto b = env->GetBoundary();
  _cdInfo.ResetVars();
  _cdInfo.m_retAllInfo = true;

  CfgType cfg = _cfg;
  CfgType dummy;

  bool valid = m_clearanceUtility.CollisionInfo(cfg, dummy, b, _cdInfo);

  if(this->m_debug) {
    cout << "CFG::" << _cfg << endl;
    cout << "ClrCfg::" << dummy << endl;
    cout << "VALID::" << valid << endl;
    cout << "Dist::" << _cdInfo.m_minDist << endl;
  }

  if(!valid || _cdInfo.m_minDist < m_obstClearance) {
    _cfg.SetLabel("VALID", false);
    return false;
  }
  else {
    _cfg.SetLabel("VALID", true);
    return true;
  }
}

/*----------------------------------------------------------------------------*/

#endif
