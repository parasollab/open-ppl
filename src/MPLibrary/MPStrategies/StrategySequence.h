#ifndef STRATEGY_SEQUENCE_H_
#define STRATEGY_SEQUENCE_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Run a number of strategies on designated tasks in sequence. The strategies
/// will not generate any output files (only this one will). Options are
/// provided to include the cost of generated paths in the stats and to clear
/// the roadmap between executions.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class StrategySequence : public MPStrategyMethod {
 public:
  ///@name Motion Planning Types
  ///@{

  typedef typename MPBaseObject::GroupCfgType GroupCfgType;
  typedef typename MPBaseObject::RoadmapType RoadmapType;
  typedef typename RoadmapType::VID VID;

  ///@}
  ///@name Local Types
  ///@{

  /// Settings for a strategy/task pair.
  struct StrategyMethod {
    std::string strategyLabel;  ///< The strategy to use.
    std::string taskLabel;      ///< The task to solve.
    bool makePath;              ///< Should we produce a path at the end?
  };

  ///@}
  ///@name Construction
  ///@{

  StrategySequence();

  StrategySequence(XMLNode& _node);

  virtual ~StrategySequence() = default;

  ///@}
  ///@name MPBaseObject Overrides
  ///@{

  virtual void Print(std::ostream& _os) const override;

  ///@}

 protected:
  ///@name MPStrategyMethod Overrides
  ///@{

  virtual void Run() override;

  ///@}
  ///@name Internal State
  ///@{

  std::vector<StrategyMethod> m_strategyMethods;

  ///@}
};

#endif