#ifndef PMPL_MP_STRATEGY_METHOD_H_
#define PMPL_MP_STRATEGY_METHOD_H_

#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPProblem/Constraints/Constraint.h"
#include "MPProblem/MPTask.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref MotionPlanningStrategies.
///
/// MPStrategyMethod has one main function, @c operator(), which
/// performs preprocessing, processing, and postprocessing functionalities of
/// the motion planning algorithm.
///
/// @usage
/// @code
/// MPStrategyPointer mps = this->GetMPStrategy(m_mpsLabel);
/// (*mps)(); //call as a function object
/// mps->operator()(); //call with pointer notation
/// @endcode
///
/// @TODO Incorporate path constraints when generating the start and goal.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPStrategyMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Construction
    ///@{

    MPStrategyMethod() = default;

    MPStrategyMethod(XMLNode& _node);

    virtual ~MPStrategyMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Interface
    ///@{

    /// Execute the strategy by calling Initialize, Run, and Finalize.
    void operator()();

    /// Set output file writing to on or off (on by default). This is used to
    /// suppress generation of roadmap, path, and stat files.
    /// @param _enable True to enable, false to disable.
    void EnableOutputFiles(const bool _enable = true);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    virtual void Initialize() {}   ///< Set up the strategy.
    virtual void Run();            ///< Call Iterate until EvaluateMap is true.
    virtual bool EvaluateMap();    ///< Check if we satisfied all map evaluators.
    virtual void Iterate() {}      ///< Execute one iteration of the strategy.
    virtual void Finalize();       ///< Clean-up and output results.

    /// Generate a 'start' node for the task and add it to the roadmap.
    /// @param _samplerLabel The label for the sampler to use if no query
    ///                      sampler was provided.
    /// @return The VID of the generated configuration.
    virtual VID GenerateStart(const std::string& _samplerLabel);

    /// Generate 'goal' node(s) for the task and add it(them) to the roadmap.
    /// @param _samplerLabel The label for the sampler to use if no query
    ///                      sampler was provided.
    /// @return The VIDs of the generated configurations.
    virtual std::vector<VID> GenerateGoals(const std::string& _samplerLabel);

    /// Virtual method used only in PRMWithRRTStrategy
    /// @TODO Remove this base-class clutter and find a more appropriate way to
    ///       implement this.
    virtual bool CheckNarrowPassageSample(VID _vid) {return false;}

    ///@}
    ///@name Internal State
    ///@{

    std::string m_querySampler;          ///< Sampler for generating start/goal.
    std::vector<std::string> m_meLabels; ///< The list of map evaluators to use.
    size_t m_iterations{0};              ///< The number of executed iterations.
    bool m_writeOutput{true};            ///< Write output at the end?

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  m_querySampler = _node.Read("querySampler", false, "", "Start/goal sampler.");

  m_writeOutput = _node.Read("writeOutput", false, m_writeOutput,
      "Enable writing of output files (roadmap, path, stats)?");

  // Parse evaluator child nodes.
  for(auto& child : _node)
    if(child.Name() == "Evaluator")
      m_meLabels.push_back(child.Read("label", true, "", "Evaluation Method"));
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel()
      << "\n\tQuery Sampler: " << m_querySampler
      << "\n\tMap Evaluators:";

  for(const auto& s : this->m_meLabels)
    _os << "\n\t\t" << s;

  _os << std::endl;
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
operator()() {
  m_iterations = 0;

  Initialize();
  Run();
  Finalize();
}


template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
EnableOutputFiles(const bool _enable) {
  m_writeOutput = _enable;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Run() {
  std::string clockName = this->GetNameAndLabel() + "::Run";
  auto stats = this->GetStatClass();
  stats->StartClock(clockName);

  Print(std::cout);

  do {
    ++m_iterations;
    if(this->m_debug)
      std::cout << "\n*** Starting iteration " << m_iterations
                << " ***************************************************"
                << "\nGraph has " << this->GetRoadmap()->GetGraph()->Size()
                << " vertices."
                << std::endl;

    Iterate();
  } while(!EvaluateMap() and this->IsRunning());

  stats->StopClock(clockName);
  stats->PrintClock(clockName, std::cout);
}


template <typename MPTraits>
bool
MPStrategyMethod<MPTraits>::
EvaluateMap() {
  // If there are no evaluators, then this is a single-iteration method.
  if(m_meLabels.empty())
    return true;

  StatClass* stats = this->GetStatClass();

  bool passed = true;
  const std::string clockName = this->GetNameAndLabel() + "::EvaluateMap";
  stats->StartClock(clockName);

  for(auto& label : m_meLabels) {
    auto evaluator = this->GetMapEvaluator(label);
    const std::string& evalName = evaluator->GetNameAndLabel();

    stats->StartClock(evalName);
    passed = evaluator->operator()();
    stats->StopClock(evalName);

    if(this->m_debug) {
      std::cout << evalName << (passed ? " Passed" : " Failed") << "\t";
      stats->PrintClock(evalName, std::cout);
      std::cout << std::endl;
    }

    if(!passed)
      break;
  }

  stats->StopClock(clockName);

  if(this->m_debug) {
    std::cout << clockName << (passed ? " Passed" : " Failed") << "\t";
    stats->PrintClock(clockName, std::cout);
    std::cout << std::endl;
  }

  return passed;
}


template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Finalize() {
  if(!m_writeOutput)
    return;

  const std::string base = this->GetBaseFilename();

  // Output final map.
  auto roadmap = this->GetRoadmap();
  roadmap->Write(base + ".map", this->GetEnvironment());

  // Output the blocked map if it is populated.
  auto blockmap = this->GetBlockRoadmap();
  if(blockmap->GetGraph()->Size())
    blockmap->Write(base + ".block.map", this->GetEnvironment());

  // Output path vertices. If you want all of the intermediates as well,
  // override this function in the derived class.
  if(this->GetPath() and this->GetPath()->Size())
    ::WritePath(base + ".rdmp.path", this->GetPath()->Cfgs());

  // Output stats.
  std::ofstream osStat(base + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, roadmap);
}


template <typename MPTraits>
typename MPStrategyMethod<MPTraits>::VID
MPStrategyMethod<MPTraits>::
GenerateStart(const std::string& _samplerLabel) {
  // If we have no start constraint, there is nothing to do.
  auto startConstraint = this->GetTask()->GetStartConstraint();
  if(!startConstraint)
    return INVALID_VID;

  // If we have no start boundary, there is also nothing to do.
  auto startBoundary = startConstraint->GetBoundary();
  if(!startBoundary)
    return INVALID_VID;

  MethodTimer mt(this->GetStatClass(), this->GetName() + "::GenerateStart");

  if(this->m_debug)
    std::cout << "Generating start configuration:" << std::endl;

  // Determine which sampler to use.
  const auto& samplerLabel = m_querySampler.empty() ? _samplerLabel
                                                    : m_querySampler;

  // Generate a valid start configuration. We will try up to 100 times before
  // giving up and declaring failure.
  std::vector<CfgType> cfgs;
  auto sampler = this->GetSampler(samplerLabel);
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
  auto g = this->GetRoadmap()->GetGraph();
  const VID vid = g->AddVertex(start);

  if(this->m_debug)
    std::cout << "\tVID " << vid << " at " << start.PrettyPrint()
              << std::endl;

  return vid;
}


template <typename MPTraits>
std::vector<typename MPStrategyMethod<MPTraits>::VID>
MPStrategyMethod<MPTraits>::
GenerateGoals(const std::string& _samplerLabel) {
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();

  // If we have no goal constraints, there is nothing to do.
  if(goalConstraints.empty())
    return {};

  MethodTimer mt(this->GetStatClass(), this->GetName() + "::GenerateGoals");

  if(this->m_debug)
    std::cout << "There are " << goalConstraints.size()
              << " goals in this problem."
              << "\nGenerating goal configurations:"
              << std::endl;

  // Determine which sampler to use.
  const auto& samplerLabel = m_querySampler.empty() ? _samplerLabel
                                                    : m_querySampler;

  auto sampler = this->GetSampler(samplerLabel);

  // Generate a valid goal configuration for each constraint. We will try up to
  // 100 times before giving up and declaring failure.
  std::vector<VID> vids;
  std::vector<CfgType> cfgs;
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
    auto g = this->GetRoadmap()->GetGraph();
    const VID vid = g->AddVertex(goal);

    vids.push_back(vid);

    if(this->m_debug)
      std::cout << "\tVID " << vid << " at " << goal.PrettyPrint()
                << std::endl;
  }

  return vids;
}

/*----------------------------------------------------------------------------*/

#endif
