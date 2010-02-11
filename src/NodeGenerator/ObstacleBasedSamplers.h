#ifndef ObstacleBasedSamplers_h
#define ObstacleBasedSamplers_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

//#include "my_program_options.hpp"
#include <sstream>

template <typename CFG>
class ObstacleBasedSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  CollisionDetection* cd;
  CDInfo *cdInfo;
  DistanceMetric* dm;
  ValidityChecker<CFG>* vc;
  std::string strVcmethod;
  std::string strLabel;
  //int n_shells_free, n_shells_coll;
  double step_size;

 public:
  ObstacleBasedSampler(Environment* _env, Stat_Class& _Stats, 
		       CollisionDetection* _cd, CDInfo& _cdInfo,
		       DistanceMetric* _dm, int _free = 1, int _coll = 0, 
		       double _step = 0) :
  env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm),step_size(_step){ 
   /* n_shells_free(_free), n_shells_coll(_coll), step_size(_step) {*/
   
    if(step_size <= 0)
      step_size = min(env->GetPositionRes(), env->GetOrientationRes());
     
}// get above parameter from XML
  int numShells,n_shells_coll,n_shells_free;
  ObstacleBasedSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
  LOG_DEBUG_MSG("ObstacleBasedSampler::ObstacleBasedSampler()");
  ParseXML(in_Node);
  //cout << "ObstacleBasedSampler";
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  
  
  vc = in_pProblem->GetValidityChecker();
  dm = in_pProblem->GetDistanceMetric();
  env = in_pProblem->GetEnvironment();
  
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  
    if(step_size <= 0)
      step_size = min(env->GetPositionRes(), env->GetOrientationRes());
     
  LOG_DEBUG_MSG("~ObstacleBasedSampler::ObstacleBasedSampler()");
  }
  // fix this later
  /*const int n_shells_free = 1, n_shells_coll = 1, numShells = 3;
  const double step_size = 1;*/
  ~ObstacleBasedSampler() {}
void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("ObstacleBasedSampler::ParseXML()");
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "n_shells_coll") {
      ParseXMLcol(*citr);
    } else if(citr->getName() == "n_shells_free") {
      ParseXMLfree(*citr);
    }else if(citr->getName() == "shells") {
      ParseXMLshells(*citr);
    }
  }
  
  
  
  cout << "ObstacleBasedSampler";
  LOG_DEBUG_MSG("~ObstacleBasedSampler::ParseXML()");
}

void ParseXMLshells(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("ObstacleBasedSampler::ParseXMLshells()");

  in_Node.verifyName(string("shells"));
  numShells = in_Node.numberXMLParameter(string("number"),true, 3,0,10,
                                         string("Number of Shells"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~ObstacleBasedSampler::ParseXMLshells()");
}

void ParseXMLcol(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("ObstacleBasedSampler::ParseXMLcol()");

  in_Node.verifyName(string("n_shells_coll"));
  n_shells_coll = in_Node.numberXMLParameter(string("number"),true, 3,0,10,
                                         string("Number of Col Shells"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~ObstacleBasedSampler::ParseXMLcol()");
}

void ParseXMLfree(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("ObstacleBasedSampler::ParseXMLfree()");

  in_Node.verifyName(string("n_shells_free"));
  n_shells_free = in_Node.numberXMLParameter(string("number"),true, 3,0,10,
                                         string("Number of Free Shells"));
  
  in_Node.warnUnrequestedAttributes();
  LOG_DEBUG_MSG("~ObstacleBasedSampler::ParseXMLfree()");
}


  const char* name() const { return "ObstacleBasedSampler"; }

  void print(ostream& os) const
    {
      os << name() 
	 << " (n_shells_free = " << n_shells_free 
	 << ", n_shells_coll = " << n_shells_coll 
	 << ", stepsize = " << step_size << ")";
    }
  
  template <typename OutputIterator>
    OutputIterator GenerateShells(Stat_Class& Stats,CFG c_free, CFG c_coll, CFG incr, 
				  OutputIterator result) 
    {
      CDInfo cdInfo;
      string callee(name());
      callee += "::GenerateShells";
     // cout << "n_shells_coll = " << n_shells_coll << endl;
      for(int i=0; i<n_shells_free; ++i) {
	if(c_free.InBoundingBox(env) && 
	  vc->IsValid(vc->GetVCMethod(strVcmethod), c_free, env, 
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
	if(c_coll.InBoundingBox(env) && 
	   !vc->IsValid(vc->GetVCMethod(strVcmethod), c_coll, env, 
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

	CFG c1 = cfg_in;
	
	bool c1_bbox = c1.InBoundingBox(env);
	
	bool c1_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c1, env, 
		         Stat, cdInfo, true, &callee);
	

	CFG c2 = c1;
	bool c2_bbox = c1_bbox;
	bool c2_free = c1_free;

	CFG r;
	r.GetRandomRay(step_size, env, dm);
	
	while(c1_bbox && c2_bbox && (c1_free == c2_free)) { 
	//while(c1_bbox && c2_bbox){
	  c1 = c2;
	  c1_bbox = c2_bbox;
	  c1_free = c2_free;
	 
	  c2.Increment(r);
	  c2_bbox = c2.InBoundingBox(env);
	
	  c2_free = vc->IsValid(vc->GetVCMethod(strVcmethod), c2, env, 
		         Stat, cdInfo, true, &callee);
	 
	}

	if(c1_bbox && c2_bbox) {
	  generated = true;
	  if(c1_free) {
	    CFG tmp;
	    r.subtract(tmp, r);
	    GenerateShells(Stat,c1, c2, r, 
			   back_insert_iterator<vector<CFG> >(cfg_out));
	  } else 
	    GenerateShells(Stat,c2, c1, r,
			   back_insert_iterator<vector<CFG> >(cfg_out));
	    }
      } while (!generated && (attempts < max_attempts));
      
      return generated;
    }
    
    template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)  
   {       CFG my_cfg;
	   do {
	   my_cfg.GetRandomCfg(env);
	   }while (!my_cfg.InBoundingBox(env));
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
