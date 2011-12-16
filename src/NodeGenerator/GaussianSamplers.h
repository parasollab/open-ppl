#ifndef GaussianSamplers_h
#define GaussianSamplers_h

#include "SamplerMethod.h"

template <typename CFG>
class GaussianSampler : public SamplerMethod<CFG>
{
  private:
    ValidityChecker<CFG>* vc;
    shared_ptr<DistanceMetricMethod> dm;
    double d;
    bool useBBX;
    string strVcmethod, dm_label;

  public:
    GaussianSampler() {
      this->SetName("GaussianSampler");
    }
    GaussianSampler(Environment* env, shared_ptr<DistanceMetric>_dm, double _d = 0) : dm(_dm), d(_d) {
      this->SetName("GaussianSampler");
      if(d == 0)
        d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
    }

    GaussianSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem):SamplerMethod<CFG>(in_Node, in_pProblem) {
      this->SetName("GaussianSampler");
      ParseXML(in_Node);
      cout << "GaussianSampler";
      cout << "strVcmethod = " << strVcmethod << endl;
      vc = in_pProblem->GetValidityChecker();
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
    }

    ~GaussianSampler() {} 

    void  ParseXML(XMLNodeReader& in_Node) {
      dm_label =in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      d = in_Node.numberXMLParameter("gauss_d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
      useBBX = in_Node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
      in_Node.warnUnrequestedAttributes();
    }

    virtual void Print(ostream& os) const {
      os << this->GetName() << " (d = " << d << ")";
    }

    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {

      string callee(this->GetName());
      callee += "::sampler()";
      bool generated = false;
      int attempts = 0;
      CDInfo cdInfo;

      CFG cfg1 = _cfgIn;
      if(cfg1 == CFG())
        cfg1.GetRandomCfg(_env);
      bool cfg1_free;
      if(!useBBX){
        if(!cfg1.InBoundingBox(_env))return false;
        cfg1_free = vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, _env,
            _stats, cdInfo, true, &callee);
      }
      else{
        cfg1_free = (cfg1.InBoundingBox(_env) &&
            vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, _env,
              _stats, cdInfo, true, &callee));
      }
      VDAddTempCfg(cfg1, cfg1_free);

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG cfg2;
        bool cfg2_free;
        if(!useBBX){
          do{
            CFG incr;
            incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), _env, dm);
            cfg2.add(cfg1, incr);
            VDAddTempRay(incr);
          }while(!cfg2.InBoundingBox(_env));
          cfg2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, 
              _stats, cdInfo, true, &callee);
        }
        else{
          CFG incr;
          incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
              _env, dm);
          cfg2.add(cfg1, incr);
          VDAddTempRay(incr);
          cfg2_free = (cfg2.InBoundingBox(_env) && 
              vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, 
                _stats, cdInfo, true, &callee));
        }
        VDAddTempCfg(cfg2, cfg2_free);
        if(cfg1_free != cfg2_free) {
          _stats.IncNodes_Generated();
          generated = true;
          ostringstream oss;
          if(cfg1_free){
            oss<<"Gaussian node generated: "<<cfg1;
            _cfgOut.push_back(cfg1);
            _cfgCol = cfg2;
          }
          else{
            oss<<"Gaussian node generated: "<<cfg2;
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
    ValidityChecker<CFG>* vc;
    shared_ptr<DistanceMetricMethod> dm;
    string dm_label;
    double d;
    string strVcmethod;
    bool use_bbx;

  public:
    BridgeTestSampler() {
      this->SetName("BridgeTestSampler");
    }

    BridgeTestSampler(Environment* env, shared_ptr<DistanceMetric> _dm, double _d = 0) : dm(_dm), d(_d) {
      this->SetName("BridgeTestSampler");
      if(d == 0)
        d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
    }

    BridgeTestSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) : SamplerMethod<CFG>(in_Node, in_pProblem) {
      this->SetName("BridgeTestSampler");
      ParseXML(in_Node);
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
      cout << "BridgeTestSampler";
      cout << "strVcmethod = " << strVcmethod << endl;
      vc = in_pProblem->GetValidityChecker();
    }

    ~BridgeTestSampler() {}

    void  ParseXML(XMLNodeReader& in_Node) {
      strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      dm_label    = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      d           = in_Node.numberXMLParameter("bridge_d",true, 0.0, 0.0,100.0,"bridge_d"); 
      use_bbx     = in_Node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
    }

    virtual void Print(ostream& os) const
    {
      os << this->GetName()
        << " (d = " << d 
        << ", use_bbx = " << use_bbx << ")" << endl;
    }

    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {

      string callee(this->GetName());
      callee += "::sampler()";
      CDInfo cdInfo;
      CFG blank_cfg;
      bool generated = false;
      int attempts = 0;

      do {
        _stats.IncNodes_Attempted();
        attempts++;
        CFG tmp = _cfgIn;
        if (tmp == blank_cfg){
          tmp.GetRandomCfg(_env);
        }
        if ( use_bbx ) {
          if ( tmp.InBoundingBox(_env) && 
              vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, _env, _stats, cdInfo, true, &callee)) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, _env, dm);
            cfg1.subtract(mid, incr);
            if ( !cfg1.InBoundingBox(_env) || 
                !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, _env, _stats, cdInfo, true, &callee)) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if(!cfg2.InBoundingBox(_env) || 
                  !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, _stats, cdInfo, true, &callee)) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(tmp);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d)), _env, dm);
            cfg2.add(cfg1, incr);
            if ( !cfg2.InBoundingBox(_env) || 
                !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, _stats, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if ( mid.InBoundingBox(_env) && 
                  (vc->IsValid(vc->GetVCMethod(strVcmethod), mid, _env, _stats, cdInfo, true, &callee))) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(mid);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          }
        } else {
          if ( vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, _env, _stats, cdInfo, true, &callee) ) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, _env, dm);
            cfg1.subtract(mid, incr);
            if( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, _env, _stats, cdInfo, true, &callee) ) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, _stats, cdInfo, true, &callee)) {
                _stats.IncNodes_Generated();
                generated = true;
                _cfgOut.push_back(tmp);
                _cfgCol = cfg1;
                //_cfgCol = cfg2;
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d)), _env, dm);
            cfg2.add(cfg1, incr);
            if ( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, _env, _stats, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if( (vc->IsValid(vc->GetVCMethod(strVcmethod), mid, _env, _stats, cdInfo, true, &callee)) ) {
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

