#ifndef MP_STRATEGY_METHOD_H_
#define MP_STRATEGY_METHOD_H_

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

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

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPProblemType::VID VID;

    MPStrategyMethod();
    MPStrategyMethod(MPProblemType* _problem, XMLNode& _node);
    virtual ~MPStrategyMethod();

    void operator()();

    virtual void Initialize() = 0;
    virtual void Run();
    virtual bool EvaluateMap();
    virtual void Iterate() {}
    virtual void Finalize() = 0;
    virtual void Print(ostream& _os) const;

    void SetBoundary(shared_ptr<Boundary> bb) {m_boundary = bb;}

    // Virtual method used in PRMWithRRTStrategy
    virtual bool CheckNarrowPassageSample(VID _vid) {return false;}

  protected:

    shared_ptr<Boundary> m_boundary;  ///< Points to the environment boundary.
    vector<string> m_meLabels;        ///< The list of map evaluators to use.
};


template<class MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod() {
}


template<class MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(MPProblemType* _problem, XMLNode& _node) :
    MPBaseObject<MPTraits>(_problem, _node) {
  if(m_boundary == NULL)
    m_boundary = this->GetEnvironment()->GetBoundary();
}


template<class MPTraits>
MPStrategyMethod<MPTraits>::
~MPStrategyMethod() {
}


template<class MPTraits>
void
MPStrategyMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}


template<class MPTraits>
void
MPStrategyMethod<MPTraits>::
operator()() {
  Initialize();
  this->Run();
  Finalize();
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
      MapEvaluatorPointer evaluator = this->GetMapEvaluator(label);
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

#endif
