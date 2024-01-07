#ifndef HIERARCHICAL_LP_H_
#define HIERARCHICAL_LP_H_

#include "LPOutput.h"
#include "LocalPlannerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief Apply a sequence of local planners until one succeeds, or all fail.
///
/// Hierarchical local planning applies a sequence of local planners until one
/// succeeds or they all fail. It will return the information of the successful
/// local plan.
////////////////////////////////////////////////////////////////////////////////
class HierarchicalLP : virtual public LocalPlannerMethod {
 public:
  /// @name Local Types
  /// @{

  typedef typename MPBaseObject::WeightType WeightType;

  ///@}
  /// @name Constructors

  HierarchicalLP(const vector<string>& _lpLabels = vector<string>(),
                 bool _saveIntermediates = false);

  HierarchicalLP(XMLNode& _node);

  ///@}
  /// @name Overrides
  /// @{

  virtual void Print(ostream& _os) const;

  virtual bool IsConnected(const Cfg& _c1,
                           const Cfg& _c2,
                           Cfg& _col,
                           LPOutput* _lpOutput,
                           double _posRes,
                           double _oriRes,
                           bool _checkCollision = true,
                           bool _savePath = false);

  ///@}

 private:
  /// @name Internal State
  /// @{

  std::vector<string> m_lpLabels;

  ///@}
};

#endif
