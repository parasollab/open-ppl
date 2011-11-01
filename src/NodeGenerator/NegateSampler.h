#ifndef NegateSampler_h
#define NegateSampler_h

#include "SamplerMethod.h"
#include "MPProblemAccess.h"

template <typename CFG> class ValidityChecker;

template <typename CFG>
class NegateSampler : public SamplerMethod<CFG> {
  ValidityChecker<CFG>* vc;
  MPProblem* mps;
  string samplingMethod;

  public:
  NegateSampler() {
    this->SetName("NegateSampler");
  }

  NegateSampler(ValidityChecker<CFG>* v, MPStrategy* s, string sm) : vc(v), mps(s), samplingMethod(sm){
    this->SetName("NegateSampler");
  }

  NegateSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
    this->SetName("NegateSampler");
    LOG_DEBUG_MSG("NegateSampler::NegateSampler()");
    ParseXML(in_Node);
    cout << "NegateSampler";
    vc = in_pProblem->GetValidityChecker();
    mps = in_pProblem;
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
    LOG_DEBUG_MSG("~NegateSampler::NegateSampler()");
  }

  ~NegateSampler() {}

  void ParseXML(XMLNodeReader& in_Node) 
  {
    LOG_DEBUG_MSG("NegateSampler::ParseXML()");
    samplingMethod = in_Node.stringXMLParameter("Method", true, "", "Sampling method to collect collision nodes for");
    //print(cout);
    cout << "NegateSampler";
    LOG_DEBUG_MSG("~NegateSampler::ParseXML()");
  }

  virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) {
    vc->ToggleValidity();
    bool result = GetSamplingMethod(mps, samplingMethod)->Sampler(_env, _stats, _cfgIn, _cfgOut, _cfgCol, _maxAttempts);
    vc->ToggleValidity();
    return result;
  }
};

#endif

