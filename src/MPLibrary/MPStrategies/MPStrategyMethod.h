#ifndef PMPL_MP_STRATEGY_METHOD_H_
#define PMPL_MP_STRATEGY_METHOD_H_

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

#ifdef VIZMO
#include "Models/Vizmo.h"
#endif

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
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPStrategyMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

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

    virtual void Initialize() {}   ///< Set up the strategy.
    virtual void Run();            ///< Call Iterate until EvaluateMap is true.
    virtual bool EvaluateMap();    ///< Check if we satisfied all map evaluators.
    virtual void Iterate() {}      ///< Execute one iteration of the strategy.
    virtual void Finalize();       ///< Clean-up and output results.

    /// Determine if any of the map evaluators have "Query" in their label.
    /// @TODO Remove this once we finish setting up the new query mechanism.
    bool UsingQuery() const;

    /// Virtual method used only in PRMWithRRTStrategy
    /// @TODO Remove this base-class clutter and find a more appropriate way to
    ///       implement this.
    virtual bool CheckNarrowPassageSample(VID _vid) {return false;}

    bool IsSuccessful() const {return m_successful;}

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::vector<std::string> m_meLabels;  ///< The list of map evaluators to use.

    /// This is currently used in Disassembly planning methods.
    bool m_successful{false}; ///< A flag to set as true in Finalize() to indicate success.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) { }

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << std::endl;
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
operator()() {
  Initialize();
  Run();
  Finalize();
#ifdef VIZMO
  GetVizmo().GetMap()->RefreshMap();
#endif
}


template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Run() {
  std::string clockName = this->GetNameAndLabel() + "::Run";
  auto stats = this->GetStatClass();
  stats->StartClock(clockName);

  Print(std::cout);

  m_successful = false;

  do {
    Iterate();
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
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
  std::string clockName = this->GetNameAndLabel() + "::EvaluateMap";
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
  // Output final map.
  auto roadmap = this->GetRoadmap();
  roadmap->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());

  // Output stats.
  std::ofstream osStat(this->GetBaseFilename() + ".stat");
  this->GetStatClass()->PrintAllStats(osStat, this->GetRoadmap());
}


template <typename MPTraits>
bool
MPStrategyMethod<MPTraits>::
UsingQuery() const {
  /// @TODO This is horribly brittle and depends on magic XML values. Let's find
  ///       a better way.
  for(const auto& label : m_meLabels)
    if(label.find("Query", 0) != std::string::npos)
      return true;
  return false;
}

/*----------------------------------------------------------------------------*/

#endif
