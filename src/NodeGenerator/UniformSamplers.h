#ifndef UNIFORMSAMPLERS_H_
#define UNIFORMSAMPLERS_H_

#include "SamplerMethod.h"

class Environment;
class Stat_Class;
class CDInfo;
template <typename CFG> class ValidityChecker;

template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG> {
  public:
    UniformRandomSampler() {
      this->SetName("UniformRandomSampler");
    }

    UniformRandomSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("UniformRandomSampler");
      ParseXML(_node);
    }

    ~UniformRandomSampler() {}

    void ParseXML(XMLNodeReader& _node) {}

    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG tmp;
        tmp.GetRandomCfg(_env);
        if(tmp.InBoundingBox(_env)) {
          _stats.IncNodes_Generated();
          generated = true;
          _cfgOut.push_back(tmp);
          _cfgCol = tmp;
        }
      } while (!generated && (attempts < _maxAttempts));

      return generated;
    }
};


template <typename CFG>
class UniformRandomFreeSampler : public SamplerMethod<CFG> {
  private:
    std::string m_vcLabel;

  public:
    UniformRandomFreeSampler() : m_vcLabel("") {
      this->SetName("UniformRandomFreeSampler");
    }

    UniformRandomFreeSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
      this->SetName("UniformRandomFreeSampler");
    } 

    UniformRandomFreeSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("UniformRandomFreeSampler");
      ParseXML(_node);
      m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    }

    ~UniformRandomFreeSampler() {}

    void ParseXML(XMLNodeReader& _node) {}

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<CFG>::PrintOptions(_out);
      _out << "\tm_vcLabel = " << m_vcLabel << endl;
    }

  protected:
    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::SampleImpl()";
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG tmp;
        tmp.GetRandomCfg(_env);
        if(tmp.InBoundingBox(_env)) {
          if(vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, 
                         _stats, cdInfo, true, &callee)) {
            _stats.IncNodes_Generated();
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

