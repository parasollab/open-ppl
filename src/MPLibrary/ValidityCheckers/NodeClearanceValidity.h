#ifndef PMPL_NODE_CLEARANCE_VALIDITY_H
#define PMPL_NODE_CLEARANCE_VALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Considers a configuration as valid if it lies outside a threshold distance
/// from all other nodes in the roadmap.
///
/// @warning This won't work correctly with batch sampling because it only
///          checks clearance against the other nodes in the roadmap. For
///          consistency it must check only one configuration at a time.
///
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class NodeClearanceValidity : virtual public ValidityCheckerMethod {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;

  ///@}
  ///@name Construction
  ///@{

  NodeClearanceValidity();

  NodeClearanceValidity(XMLNode& _node);

  virtual ~NodeClearanceValidity() = default;

  ///@}
  ///@name MPBaseObject Overrides
  ///@{

  virtual void Print(std::ostream& _os) const override;

  ///@}
  ///@name ValidityCheckerMethod Overrides
  ///@{

  virtual bool IsValidImpl(Cfg& _cfg,
                           CDInfo& _cdInfo,
                           const std::string& _callName) override;

  ///@}

 protected:
  ///@name Internal State
  ///@{

  double m_minClearance;  ///< Minimum required clearance from other nodes.
  std::string m_nfLabel;  ///< NF to find nearest nodes and distance.

  ///@}
};

#endif
