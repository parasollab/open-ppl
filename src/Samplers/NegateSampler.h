#ifndef NEGATESAMPLER_H_
#define NEGATESAMPLER_H_

#include "SamplerMethod.h"
#include "MPUtils.h"

template <typename CFG> class ValidityChecker;

template <typename CFG>
class NegateSampler : public SamplerMethod<CFG> {
  public:
    //////////////////////
    //Constructors
    //////////////////////

    NegateSampler(string _sm = "") : m_samplingMethod(_sm) {
      this->SetName("NegateSampler");
      if(this->m_debug)  
        PrintOptions(cout);
    }

    NegateSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("NegateSampler");
      ParseXML(_node);
      if(this->m_debug)
        PrintOptions(cout);
    }

    ~NegateSampler() {}

    ///////////////////
    //Accessors
    ///////////////////
    
    void 
      ParseXML(XMLNodeReader& _node) {
        m_samplingMethod = _node.stringXMLParameter("method", true, "", "Sampling method to collect collision nodes for"); 
      }

    virtual void 
      PrintOptions(ostream& _out) const {
        SamplerMethod<CFG>::PrintOptions(_out);
        _out << "\tsampling method = " << m_samplingMethod << endl;
      }
    
    virtual bool 
      Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, 
          CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol, int _maxAttempts) {

        //make invalid "valid" and vice-versa to collect collision nodes
        this->GetMPProblem()->GetValidityChecker()->ToggleValidity();  
    
        bool result = this->GetMPProblem()->GetMPStrategy()->GetSampler()->GetMethod(m_samplingMethod)
          ->Sampler(_env, _bb, _stats, _cfgIn, _cfgOut, _cfgCol, _maxAttempts);
        
        this->GetMPProblem()->GetValidityChecker()->ToggleValidity();
        
        return result;
      }

  private:
    string m_samplingMethod;
};

#endif

