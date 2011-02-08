#ifndef GaussianSamplers_h
#define GaussianSamplers_h

#include "SamplerMethod.h"
#include "util.h"


template <typename CFG, bool KEEP_FREE>
class GaussRandomSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  ValidityChecker<CFG>* vc;
  CDInfo *cdInfo;
  shared_ptr<DistanceMetricMethod> dm;
  double d;
  bool useBBX;
  string strVcmethod;

 public:
 GaussRandomSampler() {}
 GaussRandomSampler(Environment* _env, Stat_Class& _Stats, 
	                      CDInfo& _cdInfo, shared_ptr<DistanceMetric>_dm, double _d = 0) :
    env(_env), Stats(_Stats), cdInfo(_cdInfo), dm(_dm), d(_d) {
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
  
  GaussRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
  LOG_DEBUG_MSG("GaussRandomSampler::GaussRandomSampler()");
  ParseXML(in_Node);
  cout << "GaussRandomSampler";
  cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  string dm_label =in_Node.stringXMLParameter(string("dm_method"), true, string("default"), string("Distance Metric Method"));
  dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
  string strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~GaussRandomSampler::GaussRandomSampler()");
  }
 
   ~GaussRandomSampler() {} 
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GaussRandomSampler::ParseXML()");
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  d = in_Node.numberXMLParameter(string("gauss_d"), true, (double)0, (double)0, (double)MAX_DBL, string("Gaussian D value"));
  useBBX = in_Node.boolXMLParameter(string("usebbx"), true, false, string("Use bounding box as obstacle"));
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~GaussRandomSampler::ParseXML()");
 }
  
  virtual const char* name() const 
    { 
      if(KEEP_FREE)
         return "GaussRandomFreeSampler";
      else
         return "GaussRandomCollisionSampler";
    }
  
  void print(ostream& os) const
    {
      os << name() 
	 << " (d = " << d << ")";
    }
  
  bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int
  max_attempts, vector<CFG>& cfg_out_collision)
    {
      string callee(name());
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
      do {
         Stat.IncNodes_Attempted();
         attempts++;
         CFG cfg2;
         bool cfg2_free;
         if(!useBBX){
            do{
               CFG incr;
               incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
                                 env, dm);
               cfg2.add(cfg1, incr);
            }while(!cfg2.InBoundingBox(env));
            cfg2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, 
                                         Stat, cdInfo, true, &callee);
         }
         else{
            CFG incr;
            incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
                              env, dm);
            cfg2.add(cfg1, incr);
            bool cfg2_free = (cfg2.InBoundingBox(env) && 
                              vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, 
                                          Stat, cdInfo, true, &callee));
         }
         
         if(cfg1_free != cfg2_free) {
            Stat.IncNodes_Generated();
            generated = true;	
            if(cfg1_free == KEEP_FREE){
               cfg_out.push_back(cfg1);
               cfg_out_collision.push_back(cfg2);
            }
            else{
               cfg_out.push_back(cfg2);
               cfg_out_collision.push_back(cfg1);
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
        collision = copy(collision_cfg.begin(), collision_cfg.end(), result);
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
};


template <typename CFG>
class BridgeTestRandomFreeSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  //CollisionDetection* cd;
  ValidityChecker<CFG>* vc;
 // typename ValidityChecker<CFG>::VCMethodPtr vcm;
  CDInfo *cdInfo;
  shared_ptr<DistanceMetricMethod> dm;
  double d;
  std::string strVcmethod;

 public:
 BridgeTestRandomFreeSampler() {}
 BridgeTestRandomFreeSampler(Environment* _env, Stat_Class* _Stats, 
	                      CDInfo* _cdInfo, shared_ptr<DistanceMetric> _dm, double _d = 0) :
    env(_env), Stats(_Stats), cdInfo(_cdInfo), dm(_dm), d(_d) {
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
 
  BridgeTestRandomFreeSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
  LOG_DEBUG_MSG("BridgeTestRandomFreeSampler::BridgeTestRandomFreeSampler()");
  ParseXML(in_Node);
  cout << "BridgeTestRandomFreeSampler";
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  //strVcmethod = svc_method;
  cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  string dm_label = in_Node.stringXMLParameter(string("dm_method"), true, string("default"), string("Distance Metric Method"));
  dm = in_pProblem->GetDistanceMetric()->GetDMMethod(dm_label); 
 
  string strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~BridgeTestRandomFreeSampler::BridgeTestRandomFreeSampler()");
  }
 
  ~BridgeTestRandomFreeSampler() {}

  virtual const char* name() const { return "BridgeTestRandomFreeSampler"; }
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("BridgeTestRandomFreeSampler::ParseXML()");
  //print(cout);
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "bridge_d") {
      ParseXMLd(*citr);
    }
  }
  cout << "BridgeTestRandomFreeSampler";
  LOG_DEBUG_MSG("~BridgeTestRandomFreeSampler::ParseXML()");
}

void ParseXMLd(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("BridgeTestRandomFreeSampler::ParseXMLd()");

  in_Node.verifyName(string("bridge_d"));
  d = in_Node.numberXMLParameter(string("number"),true,double(0.0),
                          double(0.0),double(100000.0),string("bridge_d")); 
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~BridgeTestRandomFreeSampler::ParseXMLd()");
}

  
  void print(ostream& os) const
    {
      os << name()
	 << " (d = " << d << ")";
    }
  
  
   bool sampler(Environment* env,Stat_Class& Stat,const CFG& cfg_in, vector<CFG>& cfg_out, int
   max_attempts, vector<CFG>& cfg_collision_out)  
    {
     
      string callee(name());
      callee += "::sampler()";
      CDInfo cdInfo;
      
      
      bool generated = false;
      int attempts = 0;
      
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	CFG tmp = cfg_in;
	
	if(tmp.InBoundingBox(env) && 
	   // !tmp.isCollision(env, Stats, cd, cdInfo, true, &callee)) 
	  // vc->IsValid(vcm, tmp, env, Stat, cdInfo, true, &callee))
	vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, Stat, cdInfo, true, &callee))
	{ 
	  CFG mid = cfg_in;
	  CFG incr;
	  incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, env, dm);
	  CFG cfg1;
	  //cout << "d is = " << d<< endl;
	  cfg1.subtract(mid, incr);
	  if(!cfg1.InBoundingBox(env) || 
	       //cfg1.isCollision(env, Stats, cd, cdInfo, true, &callee))
             !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env, Stat, cdInfo, true, &callee))
	  {
		  
	    CFG cfg2;
	    cfg2.add(mid, incr);
	    if(!cfg2.InBoundingBox(env) || 
	    //   cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee))
               !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee))
	    {   //cout << "cfg2 " << cfg2 << endl;
	      Stat.IncNodes_Generated();
	      generated = true;
	     cfg_out.push_back(cfg_in);
             cfg_collision_out.push_back(cfg1);
             cfg_collision_out.push_back(cfg2);
	     
	    }
	  }
	} else {
	  CFG cfg1 = cfg_in;
	  CFG incr;
	  incr.GetRandomRay(fabs(GaussianDistribution(d, d)), env, dm);
	  CFG cfg2;
	 // cout << "cfg1 = " << cfg1 << endl;
	  cfg2.add(cfg1, incr);
	  if(!cfg2.InBoundingBox(env) || 
	    // cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee)) 
             !vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee))
	  {
	     CFG mid;
	     mid.WeightedSum(cfg1, cfg2, 0.5);
	     if(mid.InBoundingBox(env) && 
	//	!mid.isCollision(env, Stats, cd, cdInfo, true, &callee)) 
	     (vc->IsValid(vc->GetVCMethod(strVcmethod), mid, env, Stat, cdInfo, true, &callee)))
	     {
	       Stat.IncNodes_Generated();
	       generated = true;
	       cfg_out.push_back(mid);
               cfg_collision_out.push_back(cfg1);
               cfg_collision_out.push_back(cfg2);
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
    // cout << "num of nodes = " << num_nodes << endl;
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
};

#endif

