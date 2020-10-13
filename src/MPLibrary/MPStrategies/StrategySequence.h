#ifndef PMPL_STRATEGY_SEQUENCE_H_
#define PMPL_STRATEGY_SEQUENCE_H_

#include "MPStrategyMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Run a number of strategies on designated tasks in sequence. The strategies
/// will not generate any output files (only this one will). Options are
/// provided to include the cost of generated paths in the stats and to clear
/// the roadmap between executions.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class StrategySequence : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

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

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
StrategySequence<MPTraits>::
StrategySequence() {
  this->SetName("StrategySequence");
}


template <typename MPTraits>
StrategySequence<MPTraits>::
StrategySequence(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("StrategySequence");

  for(auto& child : _node) {
    if(child.Name() == "StrategyMethod") {
      StrategyMethod method;
      method.strategyLabel = child.Read("label", true, "",
          "Name of the strategy method to run.");
      method.taskLabel = child.Read("task", true, "",
          "Label of the task for this method to solve.");
      method.makePath = child.Read("path", true, true,
          "Indicates if this method is expected to generate a path.");
      m_strategyMethods.push_back(method);
    }
  }

}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
StrategySequence<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);

  _os << "\tStrategy Methods" << std::endl;
  for(const auto& method : m_strategyMethods)
    _os << "\t\t" << method.strategyLabel
        << "\tTask: " << method.taskLabel
        << "\tPath: " << method.makePath
        << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
StrategySequence<MPTraits>::
Run() {
  auto stats = this->GetStatClass();

  // Run each strategy/task in sequence.
  for(auto method : m_strategyMethods) {
    // Get the task and set it as our current one.
    auto task = this->GetMPProblem()->GetTask(method.taskLabel);
    this->GetMPLibrary()->SetTask(task);

    this->GetMPLibrary()->ResetTimeEvaluators();
    this->GetPath()->Clear();

    // Ensure the goal tracker has a goal map for this roadmap, task pair.
    auto roadmap = this->GetRoadmap(task->GetRobot());
    if(roadmap and !this->GetGoalTracker()->IsMap(roadmap, task))
      this->GetGoalTracker()->AddMap(roadmap, task);

    // Run the strategy without producing any output.
    const std::string id = method.taskLabel + "::" + method.strategyLabel;
    {
      MethodTimer mt(stats, id + "::InitAndRun");
      auto sm = this->GetMPStrategy(method.strategyLabel);
      sm->EnableOutputFiles(false);
      (*sm)();
      sm->EnableOutputFiles(true);
    }

    // If we expected a path, add its cost to the stat file.
    if(method.makePath)
      stats->SetStat(id + "::PathCost", this->GetPath()->Length());

    // Mark the task as complete.
    task->GetStatus().complete();
  }
}

/*----------------------------------------------------------------------------*/

#endif
