#ifndef NEGATESAMPLER_H_
#define NEGATESAMPLER_H_

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
  
    NegateSampler(string _sm) : samplingMethod(_sm) {
      this->SetName("NegateSampler");
    }
  
    NegateSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("NegateSampler");
      ParseXML(_node);
    }
  
    ~NegateSampler() {}
  
    void ParseXML(XMLNodeReader& _node) {
      samplingMethod = _node.stringXMLParameter("Method", true, "", "Sampling method to collect collision nodes for");
    }
  
    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, 
                         CFG& _cfgCol, int _maxAttempts) {
      this->GetMPProblem()->GetValidityChecker()->ToggleValidity();
      bool result = this->GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(samplingMethod)->Sampler(_env, _bb, _stats, _cfgIn, _cfgOut, _cfgCol, _maxAttempts);
      this->GetMPProblem()->GetValidityChecker()->ToggleValidity();
      return result;
    }

   virtual bool Sampler(Environment* _env, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
     return Sampler(_env, _env->GetBoundingBox(), _stats, _cfgIn, _cfgOut, _cfgCol, _maxAttempts);
   }
};

#endif

