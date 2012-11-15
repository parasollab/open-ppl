#ifndef MEDIALAXISSAMPLERS_H_
#define MEDIALAXISSAMPLERS_H_

#include "SamplerMethod.h"
#include "MPUtils.h"
#include <sstream>

class Environment;
class StatClass;
class CDInfo;
class DistanceMetric;

template <typename CFG>
class MedialAxisSampler : public SamplerMethod<CFG>
{
  public:
    MedialAxisSampler(const ClearanceParams& _cParams = ClearanceParams(), double _epsilon = 0.1, int _historyLength = 5):
      m_cParams(_cParams), m_epsilon(_epsilon), m_historyLength(_historyLength){
        this->SetName("MedialAxisSampler");
      }

    MedialAxisSampler(XMLNodeReader& _node, MPProblem* _problem):
      SamplerMethod<CFG>(_node, _problem),m_cParams(_node,_problem){
        this->SetName("MedialAxisSampler");
        ParseXML(_node);
      }

    ~MedialAxisSampler() {}

    void ParseXML(XMLNodeReader& _node){
      m_epsilon = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");
      m_historyLength = _node.numberXMLParameter("historyLength", false, 5, 3, 100,"History Length");
      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<CFG>::PrintOptions(_os);
      m_cParams.PrintOptions(_os);
      _os << "\tepsilon = " << m_epsilon << endl;
      _os << "\thistoryLength = " << m_historyLength << endl;
    }

    virtual string GetValidityMethod() const { return m_cParams.m_vcLabel; }
    
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, 
        StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol) {

      string call = "MedialAxisSampler::sampler()";
      bool generated = false;
      ValidityChecker* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;

      _stats.IncNodesAttempted(this->GetNameAndLabel());

      // If just a new cfg, get a random CFG
      CFG tmpCfg = _cfgIn;
      if(tmpCfg == CFG())
        tmpCfg.GetRandomCfg(_env,_bb);

      // If pushed properly and the new CFG is valid, increment generated
      if(PushToMedialAxis(tmpCfg, _stats, m_cParams, m_epsilon, m_historyLength, _bb)){
        if(vc->GetMethod(m_cParams.m_vcLabel)->IsValid(tmpCfg, _env, _stats, cdInfo, &call)) {
          _stats.IncNodesGenerated(this->GetNameAndLabel());
          generated = true;
          _cfgOut.push_back(tmpCfg);
        }
      }
      return generated;
    }

  private:
    ClearanceParams m_cParams;
    double m_epsilon;
    int m_historyLength;
};

#endif

