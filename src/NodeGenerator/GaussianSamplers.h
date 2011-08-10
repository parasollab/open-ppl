#ifndef GaussianSamplers_h
#define GaussianSamplers_h

#include "SamplerMethod.h"
#include "util.h"


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
 GaussianSampler(Environment* env, shared_ptr<DistanceMetric>_dm, double _d = 0) :
    dm(_dm), d(_d) {
    this->SetName("GaussianSampler");
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
  
  GaussianSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem):SamplerMethod<CFG>(in_Node, in_pProblem) {
  this->SetName("GaussianSampler");
  LOG_DEBUG_MSG("GaussianSampler::GaussianSampler()");
  ParseXML(in_Node);
  cout << "GaussianSampler";
  cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
  string strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~GaussianSampler::GaussianSampler()");
  }
 
   ~GaussianSampler() {} 
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GaussianSampler::ParseXML()");
  dm_label =in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
  strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
  d = in_Node.numberXMLParameter("gauss_d", true, 0.0, 0.0, MAX_DBL, "Gaussian D value");
  useBBX = in_Node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~GaussianSampler::ParseXML()");
 }
  
 virtual void print(ostream& os) const {
   os << this->GetName() 
     << " (d = " << d << ")";
 }
  
  bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int
      max_attempts, vector<CFG>& cfg_out_collision) {
    string callee(this->GetName());
    callee += "::sampler()";


    bool generated = false;
    int attempts = 0;
    CDInfo cdInfo;
    CFG cfg1 = cfg_in;
    if(cfg1 == CFG())
      cfg1.GetRandomCfg(env);
    bool cfg1_free;
    if(!useBBX){
      if(!cfg1.InBoundingBox(env))return false;
      cfg1_free = vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env,
          Stat, cdInfo, true, &callee);
    }
    else{
      cfg1_free = (cfg1.InBoundingBox(env) &&
          vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env,
            Stat, cdInfo, true, &callee));
    }
    VDAddTempCfg(cfg1, cfg1_free);
    do {
      Stat.IncNodes_Attempted();
      attempts++;
      CFG cfg2;
      bool cfg2_free;
      if(!useBBX){
        do{
          CFG incr;
          incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), env, dm);
          cfg2.add(cfg1, incr);
          VDAddTempRay(incr);
        }while(!cfg2.InBoundingBox(env));
        cfg2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, 
            Stat, cdInfo, true, &callee);
      }
      else{
        CFG incr;
        incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
            env, dm);
        cfg2.add(cfg1, incr);
        VDAddTempRay(incr);
        cfg2_free = (cfg2.InBoundingBox(env) && 
            vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, 
              Stat, cdInfo, true, &callee));
      }
      VDAddTempCfg(cfg2, cfg2_free);
      if(cfg1_free != cfg2_free) {
        Stat.IncNodes_Generated();
        generated = true;
        ostringstream oss;
        if(cfg1_free){
          oss<<"Gaussian node generated: "<<cfg1;
          cfg_out.push_back(cfg1);
          cfg_out_collision.push_back(cfg2);
        }
        else{
          oss<<"Gaussian node generated: "<<cfg2;
          cfg_out.push_back(cfg2);
          cfg_out_collision.push_back(cfg1);
        }
        VDComment(oss.str());
      }
    } while (!generated && (attempts < max_attempts));
    return generated;
  }
  
 private:
  template <typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          OutputIterator result, OutputIterator collision)  
  {
    CFG my_cfg;
    vector<CFG> out1, collision_cfg;
    for (int i =0; i< num_nodes; ++i) {
      my_cfg.GetRandomCfg(env);
      while(!sampler(env, Stat,my_cfg, out1, max_attempts, collision_cfg)) 
        my_cfg.GetRandomCfg(env);
    }
    result = copy(out1.begin(), out1.end(), result);
    collision = copy(collision_cfg.begin(), collision_cfg.end(), collision);
    return result;
  }

  template <typename InputIterator, typename OutputIterator>
  OutputIterator 
  _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
 	  OutputIterator result, OutputIterator collision)  
  {
    while(first != last) {
      vector<CFG> result_cfg, collision_cfg;
      if(sampler(env, Stat,*first,result_cfg, max_attempts, collision_cfg)){ 
        result = copy(result_cfg.begin(), result_cfg.end(), result);
        collision = copy(collision_cfg.begin(), collision_cfg.end(), collision);
      }
      first++;
    }
    return result;
  }   

 public:
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }   
  
  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
         back_insert_iterator<vector<CFG> > result)  
  {
    vector<CFG> collision;
    return _Sample(env, Stat, num_nodes, max_attempts, result, back_inserter(collision));
  }

  virtual back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 back_insert_iterator<vector<CFG> > result)  
  {
    vector<CFG> collision;
    return _Sample(env, Stat, first, last, max_attempts, result, back_inserter(collision));
  }   

  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
  {
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
  }

  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
  {
    return _Sample(env, Stat, first, last, max_attempts, result, collision);
  }
  
  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
         typename vector<CFG>::iterator result)  
  {
    vector<CFG> collision(max_attempts * num_nodes);
    return _Sample(env, Stat, num_nodes, max_attempts, result, collision.begin());
  }

  virtual typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
	 typename vector<CFG>::iterator result)  
  {
    vector<CFG> collision(max_attempts * distance(first, last));
    return _Sample(env, Stat, first, last, max_attempts, result, collision.begin());
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

    BridgeTestSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
      this->SetName("BridgeTestSampler");
      LOG_DEBUG_MSG("BridgeTestSampler::BridgeTestSampler()");
      ParseXML(in_Node);
      dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
      cout << "BridgeTestSampler";
      cout << "strVcmethod = " << strVcmethod << endl;
      vc = in_pProblem->GetValidityChecker();
      LOG_DEBUG_MSG("~BridgeTestSampler::BridgeTestSampler()");
    }

    ~BridgeTestSampler() {}

    void  ParseXML(XMLNodeReader& in_Node) {
      LOG_DEBUG_MSG("BridgeTestSampler::ParseXML()");
      strVcmethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      dm_label    = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      d           = in_Node.numberXMLParameter("bridge_d",true, 0.0, 0.0,100.0,"bridge_d"); 
			use_bbx     = in_Node.boolXMLParameter("use_bbx", false, true, "Use the Bounding Box as an Obstacle");
      string strLabel= this->ParseLabelXML( in_Node);
      this->SetLabel(strLabel);
      LOG_DEBUG_MSG("~BridgeTestRandomFreeSampler::ParseXML()");
    }

    virtual void print(ostream& os) const
    {
      os << this->GetName()
				 << " (d = " << d 
				 << ", use_bbx = " << use_bbx << ")" << endl;
    }

    bool sampler(Environment* env,Stat_Class& Stat,const CFG& cfg_in, vector<CFG>& cfg_out, int
        max_attempts, vector<CFG>& cfg_collision_out)  
    {

      string callee(this->GetName());
      callee += "::sampler()";
      CDInfo cdInfo;
      CFG blank_cfg;

      bool generated = false;
      int attempts = 0;

      do {
        Stat.IncNodes_Attempted();
        attempts++;
        CFG tmp = cfg_in;
        if (tmp == blank_cfg){
          tmp.GetRandomCfg(env);
        }
        if ( use_bbx ) {
          if ( tmp.InBoundingBox(env) && 
              vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, Stat, cdInfo, true, &callee)) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, env, dm);
            cfg1.subtract(mid, incr);
            if ( !cfg1.InBoundingBox(env) || 
                !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env, Stat, cdInfo, true, &callee)) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if(!cfg2.InBoundingBox(env) || 
                  !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee)) {
                Stat.IncNodes_Generated();
                generated = true;
                cfg_out.push_back(tmp);
                cfg_collision_out.push_back(cfg1);
                cfg_collision_out.push_back(cfg2);
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d)), env, dm);
            cfg2.add(cfg1, incr);
            if ( !cfg2.InBoundingBox(env) || 
                !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if ( mid.InBoundingBox(env) && 
                  (vc->IsValid(vc->GetVCMethod(strVcmethod), mid, env, Stat, cdInfo, true, &callee))) {
                Stat.IncNodes_Generated();
                generated = true;
                cfg_out.push_back(mid);
                cfg_collision_out.push_back(cfg1);
                cfg_collision_out.push_back(cfg2);
              }
            }
          }
        } else {
          if ( vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, Stat, cdInfo, true, &callee) ) { 
            CFG mid = tmp, incr, cfg1;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, env, dm);
            cfg1.subtract(mid, incr);
            if( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env, Stat, cdInfo, true, &callee) ) {
              CFG cfg2;
              cfg2.add(mid, incr);
              if( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee)) {
                Stat.IncNodes_Generated();
                generated = true;
                cfg_out.push_back(tmp);
                cfg_collision_out.push_back(cfg1);
                cfg_collision_out.push_back(cfg2);
              }
            }
          } else {
            CFG cfg1 = tmp, incr, cfg2;
            incr.GetRandomRay(fabs(GaussianDistribution(d, d)), env, dm);
            cfg2.add(cfg1, incr);
            if ( !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee)) {
              CFG mid;
              mid.WeightedSum(cfg1, cfg2, 0.5);
              if( (vc->IsValid(vc->GetVCMethod(strVcmethod), mid, env, Stat, cdInfo, true, &callee)) ) {
                Stat.IncNodes_Generated();
                generated = true;
                cfg_out.push_back(mid);
                cfg_collision_out.push_back(cfg1);
                cfg_collision_out.push_back(cfg2);
              }
            }
          }	
        }
      } while (!generated && (attempts < max_attempts));
      return generated;
    }

  private:
    template <typename OutputIterator>
      OutputIterator 
      _Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          OutputIterator result, OutputIterator collision)  
      {
        CFG my_cfg;
        vector<CFG> out1, collision_out;
        for (int i =0; i< num_nodes; i++){ 
          my_cfg.GetRandomCfg(env);   
          while(!sampler(env, Stat,my_cfg, out1, max_attempts, collision_out)) 
            my_cfg.GetRandomCfg(env);
        }
        result = copy(out1.begin(), out1.end(), result);
        collision = copy(collision_out.begin(), collision_out.end(), collision);
        return result;
      }

    template <typename InputIterator, typename OutputIterator>
      OutputIterator 
      _Sample(Environment* env, Stat_Class& Stat, InputIterator first, InputIterator last, int max_attempts,
          OutputIterator result, OutputIterator collision)  
      {
        while(first != last) {
          vector<CFG> result_cfg, collision_cfg;
          if(sampler(env, Stat,*first, result_cfg, max_attempts, collision_cfg)){
            result = copy(result_cfg.begin(), result_cfg.end(), result);
            collision = copy(collision_cfg.begin(), collision_cfg.end(), collision);
          }
          first++;
        }
        return result;
      }   

  public:
    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
      {
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
      }

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision)  
      {
        return _Sample(env, Stat, first, last, max_attempts, result, collision);
      }   

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          back_insert_iterator<vector<CFG> > result)  
      {
        vector<CFG> collision;
        return _Sample(env, Stat, num_nodes, max_attempts, result, back_inserter(collision));
      }

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          back_insert_iterator<vector<CFG> > result)  
      {
        vector<CFG> collision;
        return _Sample(env, Stat, first, last, max_attempts, result, back_inserter(collision));
      }   

    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
      {
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision)  
      {
        return _Sample(env, Stat, first, last, max_attempts, result, collision);
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          typename vector<CFG>::iterator result)  
      {
        vector<CFG> collision(max_attempts * num_nodes);
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision.begin());
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          typename vector<CFG>::iterator result)  
      {
        vector<CFG> collision(max_attempts * distance(first, last));
        return _Sample(env, Stat, first, last, max_attempts, result, collision.begin());
      }
};

#endif

