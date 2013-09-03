#ifndef MPSTRATEGYMETHOD_H_
#define MPSTRATEGYMETHOD_H_

#include "Utilities/MPUtils.h"
#include "Utilities/MetricUtils.h"

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
    virtual void PrintOptions(ostream& _os) const;

    string GetBaseFilename(){return m_baseFilename;}
    void SetBaseFilename(string _s){m_baseFilename = _s;}
    void SetBoundary(shared_ptr<Boundary> bb){m_boundary=bb;};

    bool EvaluateMap(vector<string> _evaluators);

    // Virtual method used in PRMWithRRTStrategy
    virtual bool CheckNarrowPassageSample(VID _vid) { return false; }

  protected:
    shared_ptr<Boundary> m_boundary;

  private:
    string m_baseFilename;
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
MPStrategyMethod<MPTraits>::PrintOptions(ostream& _os) const {
  _os << this->GetName() << endl;
}

template<class MPTraits>
void
MPStrategyMethod<MPTraits>::operator()(){
  this->GetMPProblem()->GetStatClass()->SetAuxDest(GetBaseFilename());

  Initialize();
  Run();
  Finalize();
}

template<class MPTraits>
bool
MPStrategyMethod<MPTraits>::EvaluateMap(vector<string> _evaluators) {
  if(_evaluators.empty()) {
    return true;
  }
  else {
    StatClass* stats = this->GetMPProblem()->GetStatClass();

    bool mapPassedEvaluation = false;
    string clockName = this->GetNameAndLabel() + "::EvaluateMap()";
    stats->StartClock(clockName);
    mapPassedEvaluation = true;

    for(vector<string>::iterator I = _evaluators.begin(); I != _evaluators.end(); ++I) {
      MapEvaluatorPointer evaluator = this->GetMPProblem()->GetMapEvaluator(*I);
      stringstream evaluatorClockName;
      evaluatorClockName << clockName << "::" << evaluator->GetName();
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
