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
class ObstacleClearanceValidity : virtual public ValidityCheckerMethod {
 public:
  ///@name Motion Planning Types
  ///@{

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

  virtual bool IsValidImpl(Cfg& _cfg,
                           CDInfo& _cdInfo,
                           const std::string& _callName) override;

  ///@}

 private:
  ///@name Internal State
  ///@{

  double m_clearanceThreshold;  ///< The minimum clearance threshold.
  std::string m_vcLabel;        ///< The VC label, must point to pqp solid.

  ///@}
};

#endif