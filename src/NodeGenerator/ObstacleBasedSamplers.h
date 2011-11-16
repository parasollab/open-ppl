#ifndef ObstacleBasedSamplers_h
#define ObstacleBasedSamplers_h

#include "SamplerMethod.h"
#include "Environment.h"
class Environment;
class Stat_Class;
class CDInfo;
class DistanceMetric;

//#include <sstream>

template <typename CFG>
class ObstacleBasedSampler : public SamplerMethod<CFG> {
  private:
    shared_ptr<DistanceMetricMethod >dm;
    ValidityChecker<CFG>* vc;
    std::string strVcmethod;
    int n_shells_free, n_shells_coll;
    double step_size;
    bool useBBX;

  public:
    ObstacleBasedSampler() {
      this->SetName("ObstacleBasedSampler");
    }

    ObstacleBasedSampler(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, int _free = 1, int _coll = 0, 
        double _step = 0)
      :  dm(_dm), n_shells_free(_free), n_shells_coll(_coll), step_size(_step)
    { 
      this->SetName("ObstacleBasedSampler");
      if(step_size <= 0)
        step_size = min(_env->GetPositionRes(), _env->GetOrientationRes());
    }

    ObstacleBasedSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) : SamplerMethod<CFG>(in_Node, in_pProblem) {
      this->SetName("ObstacleBasedSampler");
      ParseXML(in_Node);
      cout << "ObstacleBasedSampler";
      strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      vc = in_pProblem->GetValidityChecker();
      string dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label);
      Environment* env = in_pProblem->GetEnvironment(); 
      cout << "step_size = " << step_size << endl;
      if(step_size <= 0.0)
        step_size = min(env->GetPositionRes(), env->GetOrientationRes());
    }

    ~ObstacleBasedSampler() {}

    void  ParseXML(XMLNodeReader& in_Node) 
    {
      XMLNodeReader::childiterator citr;
      useBBX = in_Node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
      cout << "from parseXML,useBBX = " <<useBBX<< endl;
      for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
        if(citr->getName() == "n_shells_coll") {
          n_shells_coll = citr->numberXMLParameter("number",true, 3,0,10, "Number of Col Shells");
          citr->warnUnrequestedAttributes();
        } else if(citr->getName() == "n_shells_free") {
          n_shells_free = citr->numberXMLParameter("number",true, 3,0,10, "Number of Free Shells");
          citr->warnUnrequestedAttributes();
        } else if(citr->getName() == "step_size") {
          step_size = citr->numberXMLParameter("number",true, 0.0,0.0,10.0,
              "step size used in increment of cfg position towards or away from obstacles");
          citr->warnUnrequestedAttributes();
        }
      }
      cout << "ObstacleBasedSampler";
    }

    virtual void Print(ostream& os) const {
      os << this->GetName() 
        << " (n_shells_free = " << n_shells_free 
        << ", n_shells_coll = " << n_shells_coll 
        << ", stepsize = " << step_size << ")";
    }

    template <typename OutputIterator>
      OutputIterator GenerateShells(Environment* _env, Stat_Class& Stats,CFG c_free, CFG c_coll, CFG incr, 
          OutputIterator result) {
        CDInfo cdInfo;
        string callee(this->GetName());
        callee += "::GenerateShells";
        // cout << "n_shells_coll = " << n_shells_coll << endl;
        for(int i=0; i<n_shells_free; ++i) {
          if(c_free.InBoundingBox(_env) && 
              vc->IsValid(vc->GetVCMethod(strVcmethod), c_free, _env, 
                Stats, cdInfo, true, &callee))
          {
            Stats.IncNodes_Generated();
            *result = c_free;
            result++;
          }
          c_free.Increment(incr);
        }

        CFG tmp;
        // cout << "n_shells_coll = " << n_shells_coll << endl;
        incr.subtract(tmp, incr);
        for(int i=0; i<n_shells_coll; ++i) {
          if(c_coll.InBoundingBox(_env) && 
              !vc->IsValid(vc->GetVCMethod(strVcmethod), c_coll, _env, 
                Stats, cdInfo, true, &callee))
          {
            Stats.IncNodes_Generated();
            *result = c_coll;
            result++;
          }
          c_coll.Increment(incr);
        }

        return result;
      }

    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::sampler()";
      CDInfo cdInfo;
      bool generated = false;

      int attempts = 0;
      do {
        _stats.IncNodes_Attempted();
        attempts++;

        CFG c1 = _cfgIn;
        if(c1==CFG())
          c1.GetRandomCfg(_env);//random configurations taken inside bounding box

        bool c1_bbox = c1.InBoundingBox(_env);

        bool c1_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c1, _env, 
            _stats, cdInfo, true, &callee);

        CFG c2 = c1;
        bool c2_bbox = c1_bbox;
        bool c2_free = c1_free;

        CFG r;
        r.GetRandomRay(step_size, _env, dm);

        while(c2_bbox && (c1_free == c2_free)) { 
          c1 = c2;
          c1_bbox = c2_bbox;
          c1_free = c2_free;

          c2.Increment(r);
          c2_bbox = c2.InBoundingBox(_env);

          c2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c2, _env, 
              _stats, cdInfo, true, &callee);

        }
        if(c2_bbox) 
        {
          generated = true;

          if(c1_free)
          {
            CFG tmp;
            r.subtract(tmp, r);
            GenerateShells(_env, _stats,c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
            _cfgCol = c2;
          }
          else 
          {
            GenerateShells(_env, _stats,c2, c1, r,back_insert_iterator<vector<CFG> >(_cfgOut));
            _cfgCol = c1;
          }
        }
        else if(c1_bbox && useBBX && c1_free && !c2_bbox)
        {
          generated = true;
          CFG tmp;
          r.subtract(tmp, r);
          GenerateShells(_env, _stats,c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
          _cfgCol = c2;
        }
      } while (!generated && (attempts < _maxAttempts));

      return generated;
    }
};

#endif

