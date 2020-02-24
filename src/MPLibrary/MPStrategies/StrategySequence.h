#ifndef PMPL_STRATEGY_SEQUENCE_H_
#define PMPL_STRATEGY_SEQUENCE_H_

#include "MPStrategyMethod.h"

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

    /// Settings for a strategy method.
    struct StrategyMethod {
      std::string label;
      std::string task;
      bool path;
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
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Run() override;

    ///@}

  protected:

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
      method.label = child.Read("label",true,"",
          "Name of the strategy method to run.");
      method.task = child.Read("task",true,"",
          "Label of the task for this method to solve.");
      method.path = child.Read("path",true,true,
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
    _os << "\t\t" << method.label
        << "\tTask: " << method.task
        << "\tPath: " << method.path
        << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

template <typename MPTraits>
void
StrategySequence<MPTraits>::
Run() {
  auto stats = this->GetStatClass();

  for(auto method : m_strategyMethods) {

    std::string id = method.task + "::" + method.label;

    auto task = this->GetMPProblem()->GetTask(method.task);
    this->GetMPLibrary()->SetTask(task);

    auto roadmap = this->GetRoadmap(task->GetRobot());

    if(roadmap and !this->GetGoalTracker()->IsMap(roadmap,task)) {
      this->GetGoalTracker()->AddMap(roadmap,task);
    }
    this->GetPath()->Clear();

    auto sm = this->GetMPStrategy(method.label);
    MethodTimer* mt = new MethodTimer(stats, id + "::InitAndRun");
    {
      MethodTimer mt(stats, id + "::Initialize");
      sm->Initialize();
    }
    stats->PrintClock(id + "::Initialize",std::cout);
    {
      MethodTimer mt(stats, id + "::Run");
      sm->Run();
    }
    stats->PrintClock(id + "::Run", std::cout);
    delete mt;
    stats->PrintClock(id + "::InitAndRun", std::cout);
    //Do we want to allow individual strategy methods to output files or nah?
    if(method.path) {
      stats->SetStat(id + "::PathCost",
          this->GetMPSolution()->GetPath()->Length());
    }
    task->GetStatus().complete();
  }
}

/*----------------------------------------------------------------------------*/

#endif
