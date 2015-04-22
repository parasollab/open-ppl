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
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class MPStrategyMethod : public MPBaseObject<MPTraits> {
  public:

    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::MapEvaluatorPointer MapEvaluatorPointer;
    typedef typename MPProblemType::VID VID;

    MPStrategyMethod();
    MPStrategyMethod(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~MPStrategyMethod();

    void operator()();

    virtual void Initialize()=0;
    virtual void Run()=0;
    virtual void Finalize()=0;
    virtual void Print(ostream& _os) const;

    void SetBoundary(shared_ptr<Boundary> bb){m_boundary=bb;};

    bool EvaluateMap(const vector<string>& _evaluators);

    // Virtual method used in PRMWithRRTStrategy
    virtual bool CheckNarrowPassageSample(VID _vid) { return false; }

  protected:
    shared_ptr<Boundary> m_boundary;
};

template<class MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod() {
}

template<class MPTraits>
MPStrategyMethod<MPTraits>::
MPStrategyMethod(MPProblemType* _problem, XMLNodeReader& _node) :
  MPBaseObject<MPTraits>(_problem, _node) {
    if(m_boundary==NULL)
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
  Run();
  Finalize();
}

template<class MPTraits>
bool
MPStrategyMethod<MPTraits>::
EvaluateMap(const vector<string>& _evaluators) {
  if(_evaluators.empty())
    return true;
  else {
    StatClass* stats = this->GetStatClass();

    bool passed = true;
    string clockName = this->GetNameAndLabel() + "::EvaluateMap()";
    stats->StartClock(clockName);

    for(auto eval : _evaluators) {
      MapEvaluatorPointer evaluator = this->GetMapEvaluator(eval);
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
