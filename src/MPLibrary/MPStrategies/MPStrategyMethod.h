#ifndef MP_STRATEGY_METHOD_H_
#define MP_STRATEGY_METHOD_H_

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

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Interface
    ///@{

    /// Execute the strategy by calling Initialize, Run, and Finalize.
    void operator()();

    virtual void Initialize() {}   ///< Set up the strategy.
    virtual void Run();            ///< Call Iterate until EvaluateMap is true.
    virtual bool EvaluateMap();    ///< Check if we satisfied all map evaluators.
    virtual void Iterate() {}      ///< Execute one iteration of the strategy.
    virtual void Finalize() {}     ///< Clean-up and output results.

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

    vector<string> m_meLabels;        ///< The list of map evaluators to use.

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
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
operator()() {
  this->Initialize();
  this->Run();
  this->Finalize();
#ifdef VIZMO
  GetVizmo().GetMap()->RefreshMap();
#endif
}


template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Run() {
  this->Print(std::cout);

  string clockName = this->GetNameAndLabel() + "::Run";
  if(this->m_debug)
    cout << clockName << endl;
  this->GetStatClass()->StartClock(clockName);
  this->m_successful = false;

  do {
    this->Iterate();
#ifdef VIZMO
    GetVizmo().GetMap()->RefreshMap();
#endif
  } while(!this->EvaluateMap());

  this->GetStatClass()->StopClock(clockName);
  if(this->m_debug)
    this->GetStatClass()->PrintClock(clockName, cout);
}


template <typename MPTraits>
bool
MPStrategyMethod<MPTraits>::
EvaluateMap() {
  if(m_meLabels.empty())
    return true;
  else {
    StatClass* stats = this->GetStatClass();

    bool passed = true;
    string clockName = this->GetNameAndLabel() + "::EvaluateMap";
    stats->StartClock(clockName);

    for(auto& label : m_meLabels) {
      auto evaluator = this->GetMapEvaluator(label);
      const string& evalName = evaluator->GetNameAndLabel();

      stats->StartClock(evalName);
      passed = evaluator->operator()();
      stats->StopClock(evalName);

      if(this->m_debug) {
        stats->PrintClock(evalName, cout);
        cout << evalName << (passed ? "  (Passed)" : "  (Failed)") << endl;
      }

      if(!passed)
        break;
    }

    stats->StopClock(clockName);
    if(this->m_debug)
      stats->PrintClock(clockName, cout);

    return passed;
  }
}


template <typename MPTraits>
bool
MPStrategyMethod<MPTraits>::
UsingQuery() const {
  /// @TODO This is horribly brittle and depends on magic XML values. Let's find
  ///       a better way.
  for(const auto& label : m_meLabels)
    if(label.find("Query", 0) != string::npos)
      return true;
  return false;
}

/*----------------------------------------------------------------------------*/

#endif
