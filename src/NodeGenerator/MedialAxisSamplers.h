#ifndef MedialAxisSamplers_h
#define MedialAxisSamplers_h

#include "SamplerMethod.h"
#include "MedialAxisUtility.h"
#include <sstream>

class Environment;
class Stat_Class;
class CDInfo;
class DistanceMetric;

template <typename CFG>
class MedialAxisSampler : public SamplerMethod<CFG>
{
  public:
    MPProblem* mp;
    ValidityChecker<CFG>* vc;
    shared_ptr<DistanceMetricMethod> dm;
    bool m_debug,use_bbx;
    int clearance,penetration,history_len;
    string str_dm,str_vcm,str_c,str_p;
    double epsilon;

    MedialAxisSampler(MPProblem* _mp, ValidityChecker<CFG>*_vc, shared_ptr<DistanceMetricMethod> _dm, 
        bool debug, bool _use_bbx, string _str_dm, string _str_vcm, int _c, int _p, 
        string _str_c, string _str_p, double _e, int _h) : 
      mp(_mp),vc(_vc),dm(_dm),m_debug(debug),use_bbx(_use_bbx),str_dm(_str_dm),str_vcm(_str_vcm),clearance(_c),penetration(_p),str_c(_str_c),str_p(_str_p),epsilon(_e),history_len(_h) {
        this->SetName("MedialAxisSampler");
      }

    MedialAxisSampler() {
      this->SetName("MedialAxisSampler");
    }

    MedialAxisSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
      LOG_DEBUG_MSG("MedialAxisSampler::MedialAxisSampler()");
      this->SetName("MedialAxisSampler");
      ParseXML(in_Node);
      mp = in_pProblem;
      vc = in_pProblem->GetValidityChecker();
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(str_dm);
      LOG_DEBUG_MSG("~MedialAxisSampler::MedialAxisSampler()");
    }

    ~MedialAxisSampler() {}

    void ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("MedialAxisSampler::ParseXML()");		
      this->SetLabel(this->ParseLabelXML(in_Node));

      str_vcm     = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      str_dm      = in_Node.stringXMLParameter("dm_method", true, "", "Distance metric");
      str_c       = in_Node.stringXMLParameter("clearance_type", true, "", "Clearance Computation (exact or approx)");
      str_p       = in_Node.stringXMLParameter("penetration_type", true, "", "Penetration Computation (exact or approx)");		
      clearance   = in_Node.numberXMLParameter("clearance_rays", false, 10, 1, 1000, "Clearance Number");
      penetration = in_Node.numberXMLParameter("penetration_rays", false, 10, 1, 1000, "Penetration Number");
      epsilon     = in_Node.numberXMLParameter("epsilon", false, 0.1, 0.0, 1.0, "Epsilon-Close to the MA (fraction of the resolution)");		
      history_len = in_Node.numberXMLParameter("history_len", false, 5, 3, 100, "History Length");
      m_debug     = in_Node.boolXMLParameter("debug", false, false, "debugging flag");
      use_bbx     = in_Node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");		
      Print(cout);
      LOG_DEBUG_MSG("~MedialAxisSampler::ParseXML()");
    }

    virtual void Print(ostream& os) const {
      os << "MedialAxisSampler"
        << "\n  strVcmethod = " << str_vcm
        << "\n  dmstring = " << str_dm
        << "\n  epsilon = " << epsilon
        << "\n  clearance_type = " << str_c
        << "\n  penetration_type = " << str_p
        << "\n  (clearance_rays = " << clearance
        << ", penetration_rays = " << penetration
        << ")\n  use_bbx = " << use_bbx
        << "\n  history_len = " << history_len << endl;
    }

    //bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts) {
    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) {
      
      string call("MedialAxisSampler::sampler()");
      bool generated = false, c_exact, p_exact;
      int attempts = 0;
      CFG blank_cfg;
      CDInfo cdInfo;

      // Determine if performing exact computation
      c_exact = (str_c.compare("exact")==0)?true:false;
      p_exact = (str_p.compare("exact")==0)?true:false;

      do {
        _stats.IncNodes_Attempted();
        attempts++;

        // If just a new cfg, get a random CFG
        CFG tmp_cfg = _cfgIn;
        if (tmp_cfg == blank_cfg)
          tmp_cfg.GetRandomCfg(_env);

        // If pushed properly, increment generated
        if ( PushToMedialAxis(mp, _env, tmp_cfg, _stats, str_vcm, str_dm, c_exact, clearance, 
              p_exact, penetration, use_bbx, epsilon, history_len, m_debug) ) {
          if ( vc->IsValid(vc->GetVCMethod(str_vcm), tmp_cfg, _env, _stats, cdInfo, true, &call) ) {
            _stats.IncNodes_Generated();
            generated = true;
            _cfgOut.push_back(tmp_cfg);
          }
        }
      } while (!generated && (attempts < _maxAttempts));
      return generated;
    }
};

#endif

