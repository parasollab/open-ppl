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
template<class MPTraits>
class MPStrategyMethod : public MPBaseObject<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID      VID;

    ///@}
    ///\name Construction
    ///@{

    MPStrategyMethod() = default;
    MPStrategyMethod(MPProblemType* _problem, XMLNode& _node);
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
    /// \brief Designate a sampling boundary.
    /// \warning This is ignored by most strategies.
    void SetBoundary(shared_ptr<Boundary> _b) {m_boundary = _b;}

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

    shared_ptr<Boundary> m_boundary;  ///< Points to the environment boundary.
    vector<string> m_meLabels;        ///< The list of map evaluators to use.

    ///@}
};

/*----------------------------- Construction ---------------------------------*/

template<class MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(MPProblemType* _problem, XMLNode& _node) :
    MPBaseObject<MPTraits>(_problem, _node) {
  if(m_boundary == NULL)
    m_boundary = this->GetEnvironment()->GetBoundary();
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*------------------------------ Interface -----------------------------------*/

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::
operator()() {
  this->Initialize();
  this->Run();
  this->Finalize();
}


template<class MPTraits>
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


template<class MPTraits>
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
