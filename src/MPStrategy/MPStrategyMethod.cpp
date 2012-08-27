#include "MPStrategyMethod.h"
#include <sys/time.h>
#include "MPProblem.h"
#include "MapEvaluator.h"
MPStrategyMethod::MPStrategyMethod(MPSMContainer& _cont) : m_baseSeed(_cont.m_seed), m_baseFilename(_cont.m_baseFilename) {
}

MPStrategyMethod::MPStrategyMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node, _problem) {
  if(m_boundary==NULL)
    m_boundary = GetMPProblem()->GetEnvironment()->GetBoundary();
  ParseXML(_node);
}

MPStrategyMethod::~MPStrategyMethod() {
}

void MPStrategyMethod::ParseXML(XMLNodeReader& _node){
  struct timeval tv;
  gettimeofday(&tv,NULL);
  m_baseSeed = _node.numberXMLParameter("seed", false, (int)tv.tv_usec, 0, MAX_INT, "Random Seed Value"); 
  m_baseFilename = _node.stringXMLParameter("filename", true, "", "Base output filename");
};

void MPStrategyMethod::operator()(){
  SRand(m_baseSeed); 
  GetMPProblem()->GetStatClass()->SetAuxDest(GetBaseFilename());

  Initialize();
  Run();
  Finalize();
}

bool MPStrategyMethod::EvaluateMap(vector<string> _evaluators) {
  if(_evaluators.empty()) {
    return true;
  }
  else {
    StatClass* stats = GetMPProblem()->GetStatClass();

    bool mapPassedEvaluation = false;
    string clockName = this->GetNameAndLabel() + "::EvaluateMap()"; 
    stats->StartClock(clockName);
    mapPassedEvaluation = true;

    for(vector<string>::iterator I = _evaluators.begin(); I != _evaluators.end(); ++I) {
      MapEvaluator<CfgType, WeightType>::MapEvaluationPointer evaluator;
      evaluator = GetMPProblem()->GetMPStrategy()->GetMapEvaluator()->GetMethod(*I);
      stringstream evaluatorClockName; 
      evaluatorClockName << clockName << "::" << evaluator->GetName();
      stats->StartClock(evaluatorClockName.str());
      if(m_debug) cout << "\n\t";
      mapPassedEvaluation = evaluator->operator()();
      if(m_debug) cout << "\t";
      stats->StopClock(evaluatorClockName.str());
      if(m_debug){
        stats->PrintClock(evaluatorClockName.str(), cout);
      }
      if(mapPassedEvaluation){
        if(m_debug) cout << "\t  (passed)\n";
      }
      else{
        if(m_debug) cout << "\t  (failed)\n";
      }
      if(!mapPassedEvaluation)
        break;
    }
    stats->StopClock(clockName);
    if(m_debug) stats->PrintClock(clockName, cout);
    return mapPassedEvaluation;
  } 
}
