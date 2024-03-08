#include "StrategySequence.h"
#include "MPLibrary/MPLibrary.h"
/*------------------------------- Construction -------------------------------*/

StrategySequence::
StrategySequence() {
  this->SetName("StrategySequence");
}

StrategySequence::
StrategySequence(XMLNode& _node) : MPStrategyMethod(_node) {
  this->SetName("StrategySequence");

  for (auto& child : _node) {
    if (child.Name() == "StrategyMethod") {
      StrategyMethod method;
      method.strategyLabel =
          child.Read("label", true, "", "Name of the strategy method to run.");
      method.taskLabel = child.Read(
          "task", true, "", "Label of the task for this method to solve.");
      method.makePath = child.Read(
          "path", true, true,
          "Indicates if this method is expected to generate a path.");
      m_strategyMethods.push_back(method);
    }
  }
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void 
StrategySequence::
Print(std::ostream& _os) const {
  MPStrategyMethod::Print(_os);

  _os << "\tStrategy Methods" << std::endl;
  for (const auto& method : m_strategyMethods)
    _os << "\t\t" << method.strategyLabel << "\tTask: " << method.taskLabel
        << "\tPath: " << method.makePath << std::endl;
}

/*------------------------ MPStrategyMethod Overrides ------------------------*/

void 
StrategySequence::
Run() {
  auto stats = this->GetStatClass();

  // Run each strategy/task in sequence.
  for (auto method : m_strategyMethods) {
    // Get the task and set it as our current one.
    auto task = this->GetMPProblem()->GetTask(method.taskLabel);
    this->GetMPLibrary()->SetTask(task);

    this->GetMPLibrary()->ResetTimeEvaluators();
    this->GetPath()->Clear();

    // Ensure the goal tracker has a goal map for this roadmap, task pair.
    auto roadmap = this->GetRoadmap(task->GetRobot());
    if (roadmap and !this->GetMPLibrary()->GetGoalTracker()->IsMap(roadmap, task))
      this->GetMPLibrary()->GetGoalTracker()->AddMap(roadmap, task);

    // Run the strategy without producing any output.
    const std::string id = method.taskLabel + "::" + method.strategyLabel;
    {
      MethodTimer mt(stats, id + "::InitAndRun");
      auto sm = this->GetMPLibrary()->GetMPStrategy(method.strategyLabel);
      sm->EnableOutputFiles(false);
      (*sm)();
      sm->EnableOutputFiles(true);
    }

    // If we expected a path, add its cost to the stat file.
    if (method.makePath)
      stats->SetStat(id + "::PathCost", this->GetPath()->Length());

    // Mark the task as complete.
    task->GetStatus().complete();
  }
}

/*----------------------------------------------------------------------------*/