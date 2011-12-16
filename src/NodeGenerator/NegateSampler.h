#ifndef NegateSampler_h
#define NegateSampler_h

#include "SamplerMethod.h"
#include "MPUtils.h"

template <typename CFG> class ValidityChecker;

template <typename CFG>
class NegateSampler : public SamplerMethod<CFG> {
  string samplingMethod;

  public:
  NegateSampler() {
    this->SetName("NegateSampler");
  }

  NegateSampler(string sm) : samplingMethod(sm){
    this->SetName("NegateSampler");
  }

  NegateSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) : SamplerMethod<CFG>(in_Node, in_pProblem) {
    this->SetName("NegateSampler");
    ParseXML(in_Node);
  }

  ~NegateSampler() {}

  void ParseXML(XMLNodeReader& in_Node) 
  {
    samplingMethod = in_Node.stringXMLParameter("Method", true, "", "Sampling method to collect collision nodes for");
  }

  virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
    this->GetMPProblem()->GetValidityChecker()->ToggleValidity();
    bool result = GetSamplingMethod(this->GetMPProblem(), samplingMethod)->Sampler(_env, _stats, _cfgIn, _cfgOut, _cfgCol, _maxAttempts);
    this->GetMPProblem()->GetValidityChecker()->ToggleValidity();
    return result;
  }
};

#endif

