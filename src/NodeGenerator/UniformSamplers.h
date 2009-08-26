#ifndef UniformSampler_h
#define UniformSampler_h

#include "SamplerMethod.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
template <typename CFG> class ValidityChecker;

//#include "my_program_options.hpp"
#include <sstream>


template <typename CFG>
class UniformRandomSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
 // Stat_Class& Stats;  we can't do this because of XML, pointer can be NULL
  Stat_Class *Stats;
  std::string strLabel;
  
 // XMLNodeReader& in_Node;  We don't want to store this
 // MPProblem* in_pProblem;

 public:
  UniformRandomSampler() {}
  UniformRandomSampler(Environment* _env, Stat_Class& _Stats) :
    env(_env), Stats(&_Stats) {} 
  UniformRandomSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem) {
  LOG_DEBUG_MSG("UniformRandomSampler::UniformRandomSampler()");
  ParseXML(in_Node);
  cout << "UniformRandomSampler";
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
 // env = in_pProblem->GetEnvironment();
  //Stats= in_pProblem->GetStatClass();
  LOG_DEBUG_MSG("~UniformRandomSampler::UniformRandomSampler()");
  }
 
  ~UniformRandomSampler() {}

  const char* name() const { return "UniformRandomSampler"; }
  
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("UniformRandomSampler::ParseXML()");
  //print(cout);
  
  cout << "UniformRandomSampler";
  LOG_DEBUG_MSG("~UniformRandomSampler::ParseXML()");
}

  void print(ostream& os) const
    {
      os << name();
    }
    
     template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)  
    {
      LOG_DEBUG_MSG("UniformRandomSampler::UnbiasedSample()");
     /* string callee(name());
      callee += "::GenerateNode()";*/
     // cout << "num of nodes = " << num_nodes << endl;
     // for (int i =0; i< num_nodes; i++){ 
      bool generated = false;
      int attempts = 0;
      for (int i =0; i< num_nodes; i++){ 
	  cout << "Node number  = " << i << endl;
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	
	CFG tmp;
	
	tmp.GetRandomCfg(env);
	
	if(tmp.InBoundingBox(env)) {
		
	  Stat.IncNodes_Generated();
	  generated = true;
	 
	  // cfg_out.push_back(tmp);
	  *result++ = tmp;
	
	}
      } while (!generated && (attempts < max_attempts));
      
      }
      return result;
    }

   template <typename InputIterator, typename OutputIterator>
   OutputIterator Sample( Environment* env,Stat_Class& Stat, InputIterator first, InputIterator last,
	                         OutputIterator result, int max_attempts)  
    {
      LOG_DEBUG_MSG("UniformRandomSampler::BiasedSample()");
      int num_nodes;
      num_nodes = distance(first,last);
      Sample(env, Stat,num_nodes, result, max_attempts);
      return result;
    }
    
  
};


template <typename CFG>
class UniformRandomFreeSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class *Stats;
  CollisionDetection* cd;
  ValidityChecker<CFG>* vc;
  std::string strVcmethod;
  CDInfo *cdInfo;
  std::string strLabel;

 public:
 UniformRandomFreeSampler(Environment* _env, Stat_Class& _Stats, 
				CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  
  UniformRandomFreeSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
  LOG_DEBUG_MSG("UniformRandomFreeSampler::UniformRandomFreeSampler()");
  ParseXML(in_Node);
  cout << "UniformRandomFreeSampler";
  strVcmethod = in_Node.stringXMLParameter(string("vc_method"), true,
                                    string(""), string("Validity Test Method"));
  //strVcmethod = svc_method;
   cout << "strVcmethod = " << strVcmethod << endl;
  vc = in_pProblem->GetValidityChecker();
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~UniformRandomFreeSampler::UniformRandomFreeSampler()");
  }
  
  ~UniformRandomFreeSampler() {}
 void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("UniformRandomFreeSampler::ParseXML()");
  //print(cout);
  cout << "UniformRandomFreeSampler";
  LOG_DEBUG_MSG("~UniformRandomFreeSampler::ParseXML()");
}
  const char* name() const { return "UniformRandomFreeSampler"; }

  void print(ostream& os) const
    {
      os << name();
    }
  
  
    
    template <typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)   
    {
      string callee(name());
      callee += "::Sample()";
      CDInfo cdInfo;
     // cout << "num of nodes = " << num_nodes << endl;
      for (int i =0; i< num_nodes; i++){ 
      
      bool generated = false;
      int attempts = 0;
	
      do {
	Stat.IncNodes_Attempted();
	attempts++;
	  
	CFG tmp;
	//need to set label here
	//tmp.SetLabel("UniformRandomFreeSampler",true);
	tmp.GetRandomCfg(env);
	//cout << "RandomCfg::tmp = " << tmp << endl;
	if(tmp.InBoundingBox(env) && 
		 //!tmp.isCollision(env, Stat, cd, cdInfo, true, &callee)) 
	vc->IsValid(vc->GetVCMethod(strVcmethod), tmp, env, 
		         Stat, cdInfo, true, &callee))
	//cout << "num of nodes after IsValid = " << num_nodes << endl;
	{
	  Stat.IncNodes_Generated();
	  generated = true;
	  // cfg_out.push_back(tmp);
	  *result++ = tmp;
	}
      } while (!generated && (attempts < max_attempts));
      
      }
      return result;
    }
    
    template <typename InputIterator, typename OutputIterator>
   OutputIterator Sample( Environment* env,Stat_Class& Stat,InputIterator first, InputIterator last,
	                         OutputIterator result, int max_attempts)  
    {
      int num_nodes;
      num_nodes = distance(first,last);
      // cout << "Sample2 num of nodes = " << num_nodes << endl;
      Sample(env, Stat,num_nodes, result, max_attempts);
      return result;
    }
};


template <typename CFG>
class UniformRandomCollisionSampler : public SamplerMethod<CFG>
{
 private:
  Environment* env;
  Stat_Class  *Stats;
  CollisionDetection* cd;
  CDInfo  *cdInfo;
  std::string strLabel;

 public:
  UniformRandomCollisionSampler(Environment* _env, Stat_Class& _Stats, 
				CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  UniformRandomCollisionSampler(XMLNodeReader& in_Node, MPProblem* in_pProblem)
  {
  LOG_DEBUG_MSG("UniformRandomCollisionSampler::UniformRandomCollisionSampler()");
  ParseXML(in_Node);
  cd = in_pProblem->GetCollisionDetection();
  cout << "UniformRandomCollisionSampler";
  LOG_DEBUG_MSG("~UniformRandomCollisionSampler::UniformRandomCollisionSampler()");
  }
  void  ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("UniformRandomCollisionSampler::ParseXML()");
  //print(cout);
  cout << "UniformRandomCollisionSampler";
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  LOG_DEBUG_MSG("~UniformRandomCollisionSampler::ParseXML()");
}
  ~UniformRandomCollisionSampler() {}

  const char* name() const { return "UniformRandomCollisionSampler"; }

  void print(ostream& os) const
    {
      os << name();
    }

template <typename OutputIterator>
OutputIterator Sample(Environment* env,Stat_Class& Stat,int num_nodes,OutputIterator result, int max_attempts)   
    {
      string callee(name());
      callee += "::Sample()";
       CDInfo cdInfo;
      // cout << "num of nodes = " << num_nodes << endl;
      for (int i =0; i< num_nodes; i++){ 

      bool generated = false;
      int attempts = 0;
	
      do {
	   Stat.IncNodes_Attempted();
	   attempts++;
	  
	   CFG tmp;
	   tmp.GetRandomCfg(env);
	   //change to is valid
	   // cout << "tmp b4 IsValid = " << tmp << endl;
	   if(tmp.InBoundingBox(env) && tmp.isCollision(env, Stat, cd, cdInfo, true, &callee)) {
	      Stat.IncNodes_Generated();
	      // cout << "tmp after IsValid = " << tmp << endl;
	      generated = true;
	      // cfg_out.push_back(tmp);
	      *result++ = tmp;
	   }
         } while (!generated && (attempts < max_attempts));
      
      }
      return result;
    }
    
    template <typename InputIterator, typename OutputIterator>
   OutputIterator Sample(Environment* env,Stat_Class& Stat, InputIterator first, InputIterator last,
	                         OutputIterator result, int max_attempts)  
    {
      int num_nodes;
      // cout << "num of nodes = " << num_nodes << endl;
      num_nodes = distance(first,last);
      Sample(env, Stat, num_nodes, result, max_attempts);
      return result;
    }
};






#endif
