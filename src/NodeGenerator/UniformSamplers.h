#ifndef UniformSampler_h
#define UniformSampler_h

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

    UniformRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) 
    {
      this->SetName("UniformRandomSampler");
      LOG_DEBUG_MSG("UniformRandomSampler::UniformRandomSampler()");
      ParseXML(in_Node);
      cout << "UniformRandomSampler";
      string strLabel= this->ParseLabelXML( in_Node);
      this->SetLabel(strLabel);
      LOG_DEBUG_MSG("~UniformRandomSampler::UniformRandomSampler()");
    }

    ~UniformRandomSampler() {}

    void ParseXML(XMLNodeReader& in_Node) 
    {
      LOG_DEBUG_MSG("UniformRandomSampler::ParseXML()");
      SamplerMethod<CFG>::Print(cout);
      cout << "UniformRandomSampler";
      LOG_DEBUG_MSG("~UniformRandomSampler::ParseXML()");
    }

    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) {

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
    ValidityChecker<CFG>* vc;
    std::string strVcmethod;

  public:
    UniformRandomFreeSampler() {
      this->SetName("UniformRandomFreeSampler");
    }

    UniformRandomFreeSampler(ValidityChecker<CFG>* v, string vcMethod) :
      vc(v), strVcmethod(vcMethod) {
        this->SetName("UniformRandomFreeSampler");
      } 

    UniformRandomFreeSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
    {
      this->SetName("UniformRandomFreeSampler");
      LOG_DEBUG_MSG("UniformRandomFreeSampler::UniformRandomFreeSampler()");
      ParseXML(in_Node);
      cout << "UniformRandomFreeSampler";
      strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
          string(""), string("Validity Test Method"));
      cout << "strVcmethod = " << strVcmethod << endl;
      vc = in_pProblem->GetValidityChecker();
      string strLabel= this->ParseLabelXML( in_Node);
      this->SetLabel(strLabel);
      LOG_DEBUG_MSG("~UniformRandomFreeSampler::UniformRandomFreeSampler()");
    }

    ~UniformRandomFreeSampler() {}

    void ParseXML(XMLNodeReader& in_Node) 
    {
      LOG_DEBUG_MSG("UniformRandomFreeSampler::ParseXML()");
      SamplerMethod<CFG>::Print(cout);
      cout << "UniformRandomFreeSampler";
      LOG_DEBUG_MSG("~UniformRandomFreeSampler::ParseXML()");
    }

  protected:
    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) {

      string callee(this->GetName());
      callee += "::_Sample()";
      CDInfo cdInfo;
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG tmp;
        tmp.GetRandomCfg(_env);
        if(tmp.InBoundingBox(_env)){
          if(vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, _env, 
                _stats, cdInfo, true, &callee))
          {
            _stats.IncNodes_Generated();
            generated = true;
            _cfgOut.push_back(tmp);
          }
          else{
            _cfgCol = tmp;
          }
        }
      } while (!generated && (attempts < _maxAttempts));

      return generated;
    }
};

#endif

