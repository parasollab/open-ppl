#ifndef UNIFORMRANDOMSAMPLER_H_
#define UNIFORMRANDOMSAMPLER_H_

#include "SamplerMethod.h"

class Environment;
class StatClass;
class CDInfo;
template <typename CFG> class ValidityChecker;

template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG> {
  private:
    std::string m_vcLabel;

  public:
    UniformRandomSampler() : m_vcLabel("") {
      this->SetName("UniformRandomSampler");
    }

    UniformRandomSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
      this->SetName("UniformRandomSampler");
    } 

    UniformRandomSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("UniformRandomSampler");
      ParseXML(_node);
      m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    }

    ~UniformRandomSampler() {}

    void ParseXML(XMLNodeReader& _node) {}

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<CFG>::PrintOptions(_out);
      _out << "\tm_vcLabel = " << m_vcLabel << endl;
    }

  protected:
    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::SampleImpl()";
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodesAttempted();
        attempts++;
        CFG tmp;
        tmp.GetRandomCfg(_env,_bb);
        if(tmp.InBoundingBox(_env,_bb)) {
          if(vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, 
                         _stats, cdInfo, true, &callee)) {
            _stats.IncNodesGenerated();
            generated = true;
            _cfgOut.push_back(tmp);
          } else {
            _cfgCol = tmp;
          }
        }
      } while (!generated && (attempts < _maxAttempts));

      return generated;
    }  
};

#endif

