#include "MPStrategyMethod.h"

#include "MPLibrary/MPLibrary.h"

#include "ConfigurationSpace/GenericStateGraph.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Utilities/MPUtils.h"

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

/*----------------------------- Construction ---------------------------------*/

MPStrategyMethod::
MPStrategyMethod(XMLNode& _node) : MPBaseObject(_node) {
  m_querySampler = _node.Read("querySampler", false, "", "Start/goal sampler.");

  m_writeOutput = _node.Read("writeOutput", false, m_writeOutput,
      "Enable writing of output files (roadmap, path, stats)?");
  m_clearMap = _node.Read("clearMap", false, m_clearMap,
      "Clear the roadmap(s) prior to executing the strategy?");

  // Parse evaluator child nodes.
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      m_meLabels.push_back(child.Read("label", true, "", "Evaluation Method"));
}


MPStrategyMethod::
~MPStrategyMethod() { }

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
MPStrategyMethod::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tQuery Sampler: " << m_querySampler
      << "\n\tMap Evaluators:";

  for(const auto& s : this->m_meLabels)
    _os << "\n\t\t" << s;

  _os << std::endl;
}

/*------------------------------ Interface -----------------------------------*/

void
MPStrategyMethod::
operator()() {
  m_iterations = 0;

  // Print settings.
  Print(std::cout);

  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel();

  if(m_clearMap)
    ClearRoadmap();

  MethodTimer* mt = new MethodTimer(stats, id + "::InitAndRun");

  // Initialize.
  {
    MethodTimer mt(stats, id + "::Initialize");
    Initialize();
  }
  stats->PrintClock(id + "::Initialize", std::cout);

  // Run.
  Run();
  stats->PrintClock(id + "::Run", std::cout);

  delete mt;

  // Don't count finalize in the time because file io isn't relevant to
  // algorithmic performance.
  Finalize();
}


void
MPStrategyMethod::
EnableOutputFiles(const bool _enable) {
  m_writeOutput = _enable;
}

/*--------------------------------- Helpers ----------------------------------*/

void
MPStrategyMethod::
Run() {
  auto stats = this->GetStatClass();
  const std::string clockName = this->GetNameAndLabel() + "::Run";
  MethodTimer mt(stats, clockName);

  do {
    ++m_iterations;
    if(this->m_debug) {
      const size_t vertices = this->GetGroupTask()
                            ? this->GetGroupRoadmap()->Size()
                            : this->GetRoadmap()->Size();
      const std::string roadmap = this->GetGroupTask()
                                ? "Group Roadmap"
                                : "Roadmap";
      stats->PrintClock(clockName, std::cout);
      std::cout << "\n*** Starting iteration " << m_iterations
                << " ***************************************************"
                << "\n" << roadmap << " has " << vertices << " vertices."
                << std::endl;
    }

    Iterate();
  } while(!EvaluateMap() and this->IsRunning());
}


bool
MPStrategyMethod::
EvaluateMap() {
  // If there are no evaluators, then this is a single-iteration method.
  if(m_meLabels.empty())
    return true;

  const std::string clockName = this->GetNameAndLabel() + "::EvaluateMap";
  StatClass* stats = this->GetStatClass();
  stats->StartClock(clockName);

  bool passed = true;
  for(auto& label : m_meLabels) {
    auto evaluator = this->GetMPLibrary()->GetMapEvaluator(label);
    const std::string& evalName = evaluator->GetNameAndLabel();

    stats->StartClock(evalName);
    passed = evaluator->operator()();
    stats->StopClock(evalName);

    if(this->m_debug) {
      stats->PrintClock(evalName, std::cout);
      std::cout << "\t" << (passed ? "+++Passed+++" : "---Failed---")
                << std::endl;
    }

    if(!passed)
      break;
  }

  stats->StopClock(clockName);

  if(this->m_debug) {
    stats->PrintClock(clockName, std::cout);
    std::cout << "\t" << (passed ? "+++Passed+++" : "---Failed---")
              << std::endl;
  }

  return passed;
}


void
MPStrategyMethod::
Finalize() {
  if(!m_writeOutput)
    return;

  const std::string base = this->GetBaseFilename();

  // Output final map.
  auto roadmap = this->GetRoadmap();
  roadmap->Write(base + ".map", this->GetEnvironment());

  // Output the blocked map if it is populated.
  auto blockmap = this->GetBlockRoadmap();
  if(blockmap->Size())
    blockmap->Write(base + ".block.map", this->GetEnvironment());

  // Output path vertices. If you want all of the intermediates as well,
  // override this function in the derived class.
  if(this->GetPath() and this->GetPath()->Size()){
    ::WritePath(base + ".rdmp.path", this->GetPath()->Cfgs());
    ::WritePath(base + ".path", this->GetPath()->FullCfgs(this->GetMPLibrary()));
  }

  // Output stats.
  std::ofstream osStat(base + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, roadmap);

}


void
MPStrategyMethod::
ClearRoadmap() {
  /// @todo This uses the STAPL graph 'clear' function, which doesn't activate
  ///       any roadmap hooks. Methods which use hooks may have stale data after
  ///       clearing the map. To fix we'll need to replace with our own function
  ///       in RoadmapGraph.
  auto roadmap = this->GetRoadmap();
  roadmap->clear();

  // Make a new CC tracker to clear the old stale data.
  roadmap->SetCCTracker(this->GetStatClass());
}

/*--------------------------- Start/Goal Generation --------------------------*/

size_t
MPStrategyMethod::
GenerateStart(const std::string& _samplerLabel) {
  // If we have no start constraint, there is nothing to do.
  auto startConstraint = this->GetTask()->GetStartConstraint();
  if(!startConstraint)
    return INVALID_VID;

  // If we have no start boundary, there is also nothing to do.
  auto startBoundary = startConstraint->GetBoundary();
  if(!startBoundary)
    return INVALID_VID;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::GenerateStart");

  // Determine which sampler to use.
  const auto& samplerLabel = m_querySampler.empty() ? _samplerLabel
                                                    : m_querySampler;

  if(this->m_debug)
    std::cout << "Generating start configuration with sampler " << samplerLabel
              << ":"
              << std::endl;

  // Generate a valid start configuration. We will try up to 100 times before
  // giving up and declaring failure.
  std::vector<Cfg> cfgs;
  auto sampler = this->GetMPLibrary()->GetSampler(samplerLabel);
  sampler->Sample(1, 100, startBoundary, std::back_inserter(cfgs));

  // Throw an error if we failed to generate a single configuration.
  if(cfgs.empty())
    throw RunTimeException(WHERE) << "Could not generate valid start "
                                  << "configuration in boundary "
                                  << *startBoundary
                                  << " with sampler '" << samplerLabel
                                  << "'.";

  // Add the configuration to the roadmap.
  const auto& start = cfgs.front();
  auto g = this->GetRoadmap();
  const VID vid = g->AddVertex(start);

  if(this->m_debug)
    std::cout << "\tVID " << vid << " at " << start.PrettyPrint()
              << std::endl;

  // Ensure the configuration satisfies the boundary we just used to sample it.
  if(!this->GetTask()->EvaluateStartConstraints(g->GetVertex(vid)))
    throw RunTimeException(WHERE) << "Sampled configuration from a start "
                                  << "boundary, but task claims it doesn't satisfy:"
                                  << "\n\tBoundary: " << *startBoundary
                                  << "\n\tCfg:      " << start.PrettyPrint()
                                  << "\n\tFull cfg: " << start;
  // Ensure the goal tracker recognized this configuration.
  if(!this->GetMPLibrary()->GetGoalTracker()->GetStartVIDs().count(vid))
    throw RunTimeException(WHERE) << "Added VID " << vid << " as a start "
                                  << "node, but GoalTracker didn't log it.";

  return vid;
}


std::vector<size_t>
MPStrategyMethod::
GenerateGoals(const std::string& _samplerLabel) {
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();

  // If we have no goal constraints, there is nothing to do.
  if(goalConstraints.empty())
    return {};

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::GenerateGoals");

  // Determine which sampler to use.
  const auto& samplerLabel = m_querySampler.empty() ? _samplerLabel
                                                    : m_querySampler;
  auto sampler = this->GetMPLibrary()->GetSampler(samplerLabel);

  if(this->m_debug)
    std::cout << "There are " << goalConstraints.size()
              << " goals in this problem."
              << "\nGenerating goal configurations with sampler " << samplerLabel
              << ":"
              << std::endl;

  // Generate a valid goal configuration for each constraint. We will try up to
  // 100 times before giving up and declaring failure.
  std::vector<VID> vids;
  std::vector<Cfg> cfgs;
  for(size_t i = 0; i < goalConstraints.size(); ++i) {
    // Get the boundary for this goal.
    auto goalBoundary = goalConstraints[i]->GetBoundary();
    if(!goalBoundary)
      throw RunTimeException(WHERE) << "Goal " << i
                                    << " produced a null boundary.";

    // Sample a configuration.
    cfgs.clear();
    sampler->Sample(1, 100, goalBoundary, std::back_inserter(cfgs));

    // Ensure we got one.
    if(cfgs.empty())
      throw RunTimeException(WHERE) << "Could not generate valid goal " << i
                                    << " configuration in boundary "
                                    << *goalBoundary
                                    << " with sampler '"
                                    << samplerLabel << "'.";

    // Add it to the roadmap.
    auto& goal = cfgs.front();
    auto g = this->GetRoadmap();
    const VID vid = g->AddVertex(goal);

    vids.push_back(vid);

    if(this->m_debug)
      std::cout << "\tVID " << vid << " at " << goal.PrettyPrint()
                << std::endl;

    // Do not accept any bullcrap about the configuration not satisfying the
    // boundary we just used to sample it.
    if(!this->GetTask()->EvaluateGoalConstraints(g->GetVertex(vid), i))
      throw RunTimeException(WHERE) << "Sampled configuration from goal " << i
                                    << " boundary, but task claims it doesn't "
                                    << "satisfy:"
                                    << "\n\tBoundary: " << *goalBoundary
                                    << "\n\tCfg:      " << goal.PrettyPrint()
                                    << "\n\tFull cfg: " << goal;
    // Do not accept any bullcrap about the goal tracker not recognizing this
    // configuration.
    if(!this->GetMPLibrary()->GetGoalTracker()->GetGoalVIDs(i).count(vid))
      throw RunTimeException(WHERE) << "Added VID " << vid << " as a goal "
                                    << "node, but GoalTracker didn't log it.";
  }

  return vids;
}
