#ifndef MedialAxisSamplers_h
#define MedialAxisSamplers_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

#include "my_program_options.hpp"
#include <sstream>

template <typename CFG>
class FreeMedialAxisSampler : public SamplerMethod<CFG>
{
 public:
  Environment* env;
  Stat_Class *Stats;
  CollisionDetection* cd;
  ValidityChecker<CFG>* vc;
  CDInfo *cdInfo;
  DistanceMetric* dm;
  int clearance, penetration;
  std::string strLabel;
  std::string strVcmethod;

  FreeMedialAxisSampler(Environment* _env, Stat_Class& _Stats, 
			CollisionDetection* _cd, CDInfo& _cdInfo, 
			DistanceMetric* _dm, int _c, int _p) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm),
    clearance(_c), penetration(_p) {} 
  FreeMedialAxisSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
  LOG_DEBUG_MSG("FreeMedialAxisSampler::FreeMedialAxisSampler()");
  ParseXML(in_Node);
  cout << "FreeMedialAxisSampler";
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  //strVcmethod = svc_method;
   cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  dm = in_pProblem->GetDistanceMetric();
  cd = in_pProblem->GetCollisionDetection();
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  //To do:: set clearance and penetration here
  LOG_DEBUG_MSG("~FreeMedialAxisSampler::FreeMedialAxisSampler()");
  }
 
  ~FreeMedialAxisSampler() {}

  const char* name() const { return "FreeMedialAxisSampler"; }
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("FreeMedialAxisSampler::ParseXML()");
  //print(cout);
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "clearance") {
      ParseXMLclearance(*citr);
    } else if(citr->getName() == "penetration") {
      ParseXMLpenetration(*citr);
    }
  }
  cout << "FreeMedialAxisSampler";
  LOG_DEBUG_MSG("~FreeMedialAxisSampler::ParseXML()");
}

void ParseXMLclearance(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("FreeMedialAxisSampler::ParseXMLclearance()");

  in_Node.verifyName(string("clearance"));
  clearance = in_Node.numberXMLParameter(string("number"),true, 10,0,10,
                                         string("Clearance Number"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~FreeMedialAxisSampler::ParseXMLclearance()");
}

void ParseXMLpenetration(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("FreeMedialAxisSampler::ParseXMLpenetration()");

  in_Node.verifyName(string("penetration"));
  penetration = in_Node.numberXMLParameter(string("number"),true, 5,0,10,
                                         string("Penetration Number"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~FreeMedialAxisSampler::ParseXMLpenetration()");
}

  void print(ostream& os) const
    {
      os << name() 
	 << " (clearance = " << clearance 
	 << ", penetration = " << penetration << ")";
    }
  
  bool sampler(Environment* env,Stat_Class& Stat, const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)  
    {
     // LOG_DEBUG_MSG("FreeMedialAxisSampler::sampler()");
     string callee(name());
      callee += "::sampler()";
      bool generated = false;
      int attempts = 0;
      CDInfo cdInfo;
	
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	
	CFG tmp = cfg_in;
	tmp.PushToMedialAxis(env, Stat, cd, cdInfo, dm, clearance, penetration);
	if(tmp.InBoundingBox(env) && 
		vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, Stat, cdInfo, true, &callee)){
	  Stat.IncNodes_Generated();
	  generated = true;
	  cfg_out.push_back(tmp);
	}
      } while (!generated && (attempts < max_attempts));

      return generated;
    }
    
  
    template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)  
   {       CFG my_cfg;
	   vector<CFG> out1;
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
   OutputIterator Sample(Environment* env,Stat_Class& Stat, InputIterator first, InputIterator last,
	   OutputIterator result, int max_attempts) {
         while(first != last) {
    vector<CFG> result_cfg;
    if(sampler(env, Stat, *first, result_cfg, max_attempts)) {
      result = copy(result_cfg.begin(), result_cfg.end(), result);
    }
    first++;
  }
  return result;
     }
     
   
     
};

#endif
