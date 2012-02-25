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
    string m_vcLabel, m_dmLabel;
    bool m_useBBX, m_exactClearance, m_exactPenetration, m_positional;
    int m_clearanceRays, m_penetrationRays, m_historyLength;
    double m_epsilon;

    MedialAxisSampler()
      : m_vcLabel(""), m_dmLabel(""), m_useBBX(true), m_exactClearance(false), m_exactPenetration(false),
        m_clearanceRays(1), m_penetrationRays(10), m_historyLength(5), m_epsilon(0.1) {
      this->SetName("MedialAxisSampler");
    }

    MedialAxisSampler(string _vcLabel, string _dmLabel, bool _exactClearance, bool _exactPenetration, 
                      int _c = 1, int _p = 10, bool _useBBX = true, double _e = 0.1, int _h = 5)
      : m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_useBBX(_useBBX), 
        m_exactClearance(_exactClearance), m_exactPenetration(_exactPenetration),
        m_clearanceRays(_c), m_penetrationRays(_p), m_historyLength(_h), m_epsilon(_e) {
      this->SetName("MedialAxisSampler");
    }

    MedialAxisSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("MedialAxisSampler");
      ParseXML(_node);
    }

    ~MedialAxisSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      m_dmLabel = _node.stringXMLParameter("dm_method", true, "", "Distance metric");

      m_clearanceRays = _node.numberXMLParameter("clearance_rays", false, 10, 1, 1000, "Clearance Number");
      string clearanceType = _node.stringXMLParameter("clearance_type", true, "", "Clearance Computation (exact or approx)");
      m_exactClearance = (clearanceType.compare("exact")==0) ? true : false;
      
      m_penetrationRays = _node.numberXMLParameter("penetration_rays", false, 10, 1, 1000, "Penetration Number");
      string penetrationType = _node.stringXMLParameter("penetration_type", true, "", "Penetration Computation (exact or approx)");
      m_exactPenetration = (penetrationType.compare("exact")==0) ? true : false;
      
      m_epsilon = _node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");
      m_historyLength = _node.numberXMLParameter("history_len", false, 5, 3, 100, "History Length");
      m_useBBX = _node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
      m_positional = _node.boolXMLParameter("positional", false, true, "Use only positional DOFs");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<CFG>::PrintOptions(_os);
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
      _os << "\tuseBBX = " << m_useBBX << endl;
      _os << "\tclearance = ";
      if(m_exactClearance == true)
        _os << "exact, ";
      else
        _os << "approx, ";
      _os << m_clearanceRays << " rays\n";
      _os << "\tpenetration = ";
      if(m_exactPenetration == true)
        _os << "exact, ";
      else
        _os << "approx, ";
      _os << m_penetrationRays << " rays\n";
      _os << "\tepsilon = " << m_epsilon << endl;
      _os << "\thistoryLength = " << m_historyLength << endl;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol, int _maxAttempts) {
      string call("MedialAxisSampler::sampler()");
      bool generated = false;
      int attempts = 0;
      CFG blankCfg;
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;

      do {
        _stats.IncNodesAttempted(this->GetNameAndLabel());
        attempts++;

        // If just a new cfg, get a random CFG
        CFG tmpCfg = _cfgIn;
        if(tmpCfg == blankCfg)
          tmpCfg.GetRandomCfg(_env,_bb);

        // If pushed properly, increment generated
        if(PushToMedialAxis(this->GetMPProblem(), _env, _bb, tmpCfg, _stats, m_vcLabel, m_dmLabel, 
                            m_exactClearance, m_clearanceRays, m_exactPenetration, m_penetrationRays, 
                            m_useBBX, m_epsilon, m_historyLength, this->m_debug, m_positional)) {
          if(vc->IsValid(vc->GetVCMethod(m_vcLabel), tmpCfg, _env, _stats, cdInfo, true, &call)) {
            _stats.IncNodesGenerated(this->GetNameAndLabel());
            generated = true;
            _cfgOut.push_back(tmpCfg);
          }
        }
      } while (!generated && (attempts < _maxAttempts));
      return generated;
    }
};

#endif

