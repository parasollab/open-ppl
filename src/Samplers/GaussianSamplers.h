#ifndef GAUSSIANSAMPLERS_H_
#define GAUSSIANSAMPLERS_H_

#include "SamplerMethod.h"

template <typename CFG>
class GaussianSampler : public SamplerMethod<CFG>
{
  private:
    double m_d;
    bool m_useBoundingBox;
    string m_vcLabel, m_dmLabel;

  public:
    GaussianSampler() : m_d(0), m_useBoundingBox(false), m_vcLabel(""), m_dmLabel("") {
      this->SetName("GaussianSampler");
    }
    
    GaussianSampler(Environment* _env, string _vcLabel, string _dmLabel, double _d = 0, bool _useBoundingBox = false)
      : m_d(_d), m_useBoundingBox(_useBoundingBox), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
      this->SetName("GaussianSampler");
      if(m_d == 0)
        m_d = (_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange();
    }

    GaussianSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("GaussianSampler");
      ParseXML(_node);
    }

    ~GaussianSampler() {} 

    void  ParseXML(XMLNodeReader& _node) {
      m_d = _node.numberXMLParameter("gauss_d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
      m_useBoundingBox = _node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
      m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      m_dmLabel =_node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      
      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<CFG>::PrintOptions(_out);
      _out << "\td = " << m_d << endl; 
      _out << "\tuseBoundingBox = " << m_useBoundingBox << endl; 
      _out << "\tvcLabel = " << m_vcLabel << endl; 
      _out << "\tdmLabel = " << m_dmLabel << endl; 
    }

    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::sampler()";
      bool generated = false;
      int attempts = 0;
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;
      shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dmLabel); 

      CFG cfg1 = _cfgIn;
      if(cfg1 == CFG())
        cfg1.GetRandomCfg(_env,_bb);
      bool cfg1Free;
      if(!m_useBoundingBox) {
        if(!cfg1.InBoundingBox(_env,_bb))
          return false;
        cfg1Free = vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env,
                               _stats, cdInfo, true, &callee);
      } else {
        cfg1Free = (cfg1.InBoundingBox(_env,_bb) &&
                    vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env,
                                _stats, cdInfo, true, &callee));
      }
      VDAddTempCfg(cfg1, cfg1Free);

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG cfg2;
        bool cfg2Free;
        if(!m_useBoundingBox) {
          do {
            CFG incr;
            incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
            cfg2.add(cfg1, incr);
            VDAddTempRay(incr);
          } while(!cfg2.InBoundingBox(_env,_bb));
          cfg2Free = vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, 
                                 _stats, cdInfo, true, &callee);
        } else {
          CFG incr;
          incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), _env, dm);
          cfg2.add(cfg1, incr);
          VDAddTempRay(incr);
          cfg2Free = (cfg2.InBoundingBox(_env,_bb) && 
                      vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, 
                                  _stats, cdInfo, true, &callee));
        }
        VDAddTempCfg(cfg2, cfg2Free);
        if(cfg1Free != cfg2Free) {
          _stats.IncNodes_Generated();
          generated = true;
          ostringstream oss;
          if(cfg1Free) {
            oss << "Gaussian node generated: " << cfg1;
            _cfgOut.push_back(cfg1);
            _cfgCol = cfg2;
          } else {
            oss << "Gaussian node generated: " << cfg2;
            _cfgOut.push_back(cfg2);
            _cfgCol = cfg1;
          }
          VDComment(oss.str());
        }
      } while (!generated && (attempts < _maxAttempts));
      return generated;
    }
};


template <typename CFG>
class BridgeTestSampler : public SamplerMethod<CFG>
{
  private:
    double m_d;
    bool m_useBoundingBox;
    string m_vcLabel, m_dmLabel;

  public:
    BridgeTestSampler() : m_d(0), m_useBoundingBox(false), m_vcLabel(""), m_dmLabel("") {
      this->SetName("BridgeTestSampler");
    }

    BridgeTestSampler(Environment* _env, string _vcLabel, string _dmLabel, double _d = 0, bool _useBoundingBox = false)
      : m_d(_d), m_useBoundingBox(_useBoundingBox), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel) {
      this->SetName("BridgeTestSampler");
      if(m_d == 0)
        m_d = (_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange();
    }

    BridgeTestSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("BridgeTestSampler");
      ParseXML(_node);
    }

    ~BridgeTestSampler() {}

    void  ParseXML(XMLNodeReader& _node) {
      m_d = _node.numberXMLParameter("bridge_d",true, 0.0, 0.0,100.0,"bridge_d"); 
      m_useBoundingBox = _node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
      m_vcLabel = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      m_dmLabel = _node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<CFG>::PrintOptions(_out);
      _out << "\td = " << m_d << endl; 
      _out << "\tuseBoundingBox = " << m_useBoundingBox << endl; 
      _out << "\tvcLabel = " << m_vcLabel << endl; 
      _out << "\tdmLabel = " << m_dmLabel << endl; 
    }

    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::sampler()";
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;
      shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dmLabel); 
      CFG blankCfg;
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG tmp = _cfgIn;
        if (tmp == blankCfg) {
          tmp.GetRandomCfg(_env,_bb);
        }
        if ( m_useBoundingBox ) {
          if ( tmp.InBoundingBox(_env,_bb) && 
              vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, _stats, cdInfo, true, &callee)) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
            cfg1.subtract(mid, incr);
            if ( !cfg1.InBoundingBox(_env,_bb) || 
                !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env, _stats, cdInfo, true, &callee)) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if(!cfg2.InBoundingBox(_env,_bb) || 
                 !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(tmp);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
            cfg2.add(cfg1, incr);
            if ( !cfg2.InBoundingBox(_env,_bb) || 
                !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if ( mid.InBoundingBox(_env,_bb) && 
                  (vc->IsValid(vc->GetVCMethod(m_vcLabel), mid, _env, _stats, cdInfo, true, &callee))) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(mid);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          }
        } else {
          if ( vc->IsValid(vc->GetVCMethod(m_vcLabel), tmp, _env, _stats, cdInfo, true, &callee) ) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d))/2, _env, dm);
            cfg1.subtract(mid, incr);
            if( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg1, _env, _stats, cdInfo, true, &callee) ) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(tmp);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(m_d, m_d)), _env, dm);
            cfg2.add(cfg1, incr);
            if ( !vc->IsValid(vc->GetVCMethod(m_vcLabel), cfg2, _env, _stats, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if( (vc->IsValid(vc->GetVCMethod(m_vcLabel), mid, _env, _stats, cdInfo, true, &callee)) ) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(mid);
                _cfgCol = cfg1;
                //cfgCol = cfg2;
              }
            }
          }	
        }
      } while (!generated && (attempts < _maxAttempts));
      return generated;
    }
};

#endif

