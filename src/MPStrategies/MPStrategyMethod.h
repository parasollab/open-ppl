#ifndef MPSTRATEGYMETHOD_H_
#define MPSTRATEGYMETHOD_H_

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
/// MPStrategyPointer mps = this->GetMPProblem()->GetMPStrategy(m_mpsLabel);
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

    virtual void ParseXML(XMLNodeReader& _node);

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
MPStrategyMethod<MPTraits>::MPStrategyMethod() {
}

template<class MPTraits>
MPStrategyMethod<MPTraits>::MPStrategyMethod(MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node) {
  ParseXML(_node);
  if(m_boundary==NULL)
    m_boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();
}

template<class MPTraits>
MPStrategyMethod<MPTraits>::~MPStrategyMethod() {
}

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::ParseXML(XMLNodeReader& _node){
};

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::operator()(){
  Initialize();
  Run();
  Finalize();
}

template<class MPTraits>
bool
MPStrategyMethod<MPTraits>::EvaluateMap(const vector<string>& _evaluators) {
  if(_evaluators.empty())
    return true;
  else {
    StatClass* stats = this->GetMPProblem()->GetStatClass();

    bool mapPassedEvaluation = false;
    string clockName = this->GetNameAndLabel() + "::EvaluateMap()";
    stats->StartClock(clockName);
    mapPassedEvaluation = true;

    for(vector<string>::const_iterator I = _evaluators.begin(); I != _evaluators.end(); ++I) {
      MapEvaluatorPointer evaluator = this->GetMPProblem()->GetMapEvaluator(*I);
      stringstream evaluatorClockName;
      evaluatorClockName << clockName << "::" << evaluator->GetNameAndLabel();
      stats->StartClock(evaluatorClockName.str());
      if(this->m_debug) cout << "\n\t";
      mapPassedEvaluation = evaluator->operator()();
      if(this->m_debug) cout << "\t";
      stats->StopClock(evaluatorClockName.str());
      if(this->m_debug){
        stats->PrintClock(evaluatorClockName.str(), cout);
      }
      if(mapPassedEvaluation){
        if(this->m_debug) cout << "\t  (passed)\n";
      }
      else{
        if(this->m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    stats->StopClock(clockName);
    if(this->m_debug) stats->PrintClock(clockName, cout);
    return mapPassedEvaluation;
  }
}

#endif
