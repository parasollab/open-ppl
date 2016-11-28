#ifndef MP_STRATEGY_METHOD_H_
#define MP_STRATEGY_METHOD_H_

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

#ifdef VIZMO
#include "Models/Vizmo.h"
#endif

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Base algorithm abstraction for \ref MotionPlanningStrategies.
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
/// \todo Once all MPStrategies have been configured to support pausible
///       execution, the Iterate() method should be made pure virtual to reflect
///       the lack of an appropriate default behavior. We will leave it with an
///       empty implementation for now to give people time to update their code.
///       We can then remove documentation notes related to this change.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MPStrategyMethod : public MPBaseObject<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///\name Construction
    ///@{

    MPStrategyMethod() = default;
    MPStrategyMethod(XMLNode& _node);
    virtual ~MPStrategyMethod() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name Interface
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Execute the strategy by calling Initialize, Run, and Finalize.
    void operator()();

    virtual void Initialize() = 0; ///< Set up the strategy.
    virtual void Run();            ///< Call Iterate until EvaluateMap is true.
    virtual bool EvaluateMap();    ///< Check if we satisfied all map evaluators.
    virtual void Iterate() {}      ///< Execute one iteration of the strategy.
    virtual void Finalize() = 0;   ///< Clean-up and output results.

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Determine if any of the map evaluators have "Query" in their
    ///        label.
    bool UsingQuery() const;

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Virtual method used only in PRMWithRRTStrategy
    virtual bool CheckNarrowPassageSample(VID _vid) {return false;}

    ///@}

  protected:

    ///\name Internal State
    ///@{

    vector<string> m_meLabels;        ///< The list of map evaluators to use.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

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
}


template <typename MPTraits>
void
MPStrategyMethod<MPTraits>::
Run() {
  string clockName = this->GetNameAndLabel() + "::Run()";
  if(this->m_debug)
    cout << clockName << endl;
  this->GetStatClass()->StartClock(clockName);

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
    string clockName = this->GetNameAndLabel() + "::EvaluateMap()";
    stats->StartClock(clockName);

    for(auto& label : m_meLabels) {
      auto evaluator = this->GetMapEvaluator(label);
      const string& evalName = evaluator->GetNameAndLabel();

      stats->StartClock(evalName);
      passed = evaluator->operator()();
      stats->StopClock(evalName);

      if(this->m_debug) {
        stats->PrintClock(evaluator->GetNameAndLabel(), cout);
        cout << evalName
             << (passed ? "  (Passed)" : "  (Failed)") << endl;
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
  for(const auto& label : m_meLabels)
    if(label.find("Query", 0) != string::npos)
      return true;
  return false;
}

/*----------------------------------------------------------------------------*/

#endif
