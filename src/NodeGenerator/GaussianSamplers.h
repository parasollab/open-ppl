#ifndef GaussianSamplers_h
#define GaussianSamplers_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;
template <typename CFG> class ValidityChecker;
#include "my_program_options.hpp"
#include <sstream>
#include <string>
#include "util.h"


template <typename CFG, bool KEEP_FREE>
class GaussRandomSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
 // Stat_Class& Stats;
 Stat_Class *Stats;
//  CollisionDetection* cd;
  ValidityChecker<CFG>* vc;
 // typename ValidityChecker<CFG>::VCMethodPtr vcm;
  CDInfo *cdInfo;
  DistanceMetric* dm;
  double d;
  std::string strLabel;
  std::string strVcmethod;

  
  DistanceMetricMethod* m_dmm;
 public:
 GaussRandomSampler(Environment* _env, Stat_Class& _Stats, 
	                      CDInfo& _cdInfo, DistanceMetric* _dm, double _d = 0) :
    env(_env), Stats(_Stats), cdInfo(_cdInfo), dm(_dm), d(_d) {
    m_dmm = new EuclideanDistance();
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
  
  GaussRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
  LOG_DEBUG_MSG("GaussRandomSampler::GaussRandomSampler()");
  ParseXML(in_Node);
  cout << "GaussRandomSampler";
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  //strVcmethod = svc_method;
   cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  dm = in_pProblem->GetDistanceMetric();
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~GaussRandomSampler::GaussRandomSampler()");
  }
 
   ~GaussRandomSampler() {} 
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GaussRandomSampler::ParseXML()");
  //print(cout);
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "gauss_d") {
      ParseXMLd(*citr);
    }
  }
  cout << "GaussRandomSampler";
  LOG_DEBUG_MSG("~GaussRandomSampler::ParseXML()");
}

void ParseXMLd(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GaussRandomSampler::ParseXMLd()");

  in_Node.verifyName(string("gauss_d"));
  d = in_Node.numberXMLParameter(string("number"),true,double(0.0),
                          double(0.0),double(100000.0),string("gauss_d")); 
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~GaussRandomSampler::ParseXMLd()");
}
  
  const char* name() const 
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
  
  bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)
    {
      string callee(name());
      callee += "::sampler()";
     
            
      bool generated = false;
      int attempts = 0;
      CDInfo cdInfo;
	
      CFG cfg1 = cfg_in;
     // cout << "d is = " << d<< endl;
      bool cfg1_free = (cfg1.InBoundingBox(env) && 
	// !cfg1.isCollision(env, Stats, cd, cdInfo, true, &callee);
	vc->IsValid(vc->GetVCMethod(strVcmethod), cfg1, env, 
		         Stat, cdInfo, true, &callee));
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	  
	CFG incr;
	incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
			      env, dm);
	CFG cfg2;
	//cout << "cfg2 is = " << cfg2 << endl;
	cfg2.add(cfg1, incr);
	bool cfg2_free = (cfg2.InBoundingBox(env) && 
//	  !cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee);
	  vc->IsValid(vc->GetVCMethod(strVcmethod), cfg2, env, Stat, cdInfo, true, &callee));

	if(cfg1_free != cfg2_free) {
	  Stat.IncNodes_Generated();
	  generated = true;	
	  if(cfg1_free == KEEP_FREE)
	   cfg_out.push_back(cfg1);
          //*result++ = cfg1;
	  else
	    cfg_out.push_back(cfg2);
           //*result++ = cfg2;
	}
      } while (!generated && (attempts < max_attempts));
    
      return generated;
   }
   
   template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)  
   {       
	   CFG my_cfg;
	   vector<CFG> out1;
	  
	   for (int i =0; i< num_nodes; ++i){ 
		my_cfg.GetRandomCfg(env);   
	       while(!sampler(env, Stat,my_cfg, out1, max_attempts)) {
	     
		  my_cfg.GetRandomCfg(env);
               }	   
	   
	  }
	  result = copy(out1.begin(), out1.end(), result);
	  return result;
    }
   
   
   template <typename InputIterator, typename OutputIterator>
   OutputIterator Sample( Environment* env,Stat_Class& Stat,InputIterator first, InputIterator last,
	   OutputIterator result, int max_attempts) {
  
         while(first != last) {
    CFG cfg_in = *first;
    vector<CFG> result_cfg;
    cfg_in.GetRandomCfg(env);
     if(sampler(env, Stat,cfg_in, result_cfg, max_attempts)) {
    
      result = copy(result_cfg.begin(), result_cfg.end(), result);
     
    }
    first++;
  }
  return result;
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
  DistanceMetric* dm;
  double d;
  std::string strLabel;
  std::string strVcmethod;

 public:
 BridgeTestRandomFreeSampler(Environment* _env, Stat_Class& _Stats, 
	                      CDInfo& _cdInfo, DistanceMetric* _dm, double _d = 0) :
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
  dm = in_pProblem->GetDistanceMetric();
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~BridgeTestRandomFreeSampler::BridgeTestRandomFreeSampler()");
  }
 
  ~BridgeTestRandomFreeSampler() {}

  const char* name() const { return "BridgeTestRandomFreeSampler"; }
  
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
  
  
   bool sampler(Environment* env,Stat_Class& Stat,const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)  
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
	     }
	  }
	}
      } while (!generated && (attempts < max_attempts));
     
      return generated;
    }
    
     template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)  
   {       
	   CFG my_cfg;
	   vector<CFG> out1;
	  // cout << "num of nodes = " << num_nodes << endl;
	   for (int i =0; i< num_nodes; i++){ 
		my_cfg.GetRandomCfg(env);   
	       while(!sampler(env, Stat,my_cfg, out1, max_attempts)) {
		       my_cfg.GetRandomCfg(env);
                  
               }	   
	   
	  }
	  result = copy(out1.begin(), out1.end(), result);
	  return result;
    }
   
   
   template <typename InputIterator, typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,InputIterator first, InputIterator last,
	   OutputIterator result, int max_attempts) {
  
         while(first != last) {
    
    vector<CFG> result_cfg;
    if(sampler(env, Stat,*first, result_cfg, max_attempts)) {
      result = copy(result_cfg.begin(), result_cfg.end(), result);
    }
    first++;
  }
  return result;
     }
     
    
    
     
};

#endif
