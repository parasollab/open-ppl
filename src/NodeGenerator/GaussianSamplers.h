#ifndef GaussianSamplers_h
#define GaussianSamplers_h

#include "BaseSampler.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

#include "my_program_options.hpp"
#include <sstream>

double
GaussianDistribution(double m, double s) 
{
  double x1, x2, w, r, r1;
  do {
    r1 = (double)rand();
    r = ((double)(r1 / (RAND_MAX+1))) * -1;
    x1 = 2. * r - 1.;
    r1 = (double)rand();
    r = ((double)(r1 / (RAND_MAX+1))) * -1;
    x2 = 2. * r - 1.;
    w = x1 * x1 + x2 * x2;
  } while(w >= 1. || w < 1E-30);
  w = sqrt((-2. * log(w)) / w);
  x1 *= w;
  return x1 * s + m;
}


template <typename CFG, bool KEEP_FREE>
class GaussRandomSampler : public BaseSampler<CFG> 
{
 private:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;
  DistanceMetric* dm;
  double d;

 public:
  GaussRandomSampler(Environment* _env, Stat_Class& _Stats, 
		     CollisionDetection* _cd, CDInfo& _cdInfo, 
		     DistanceMetric* _dm, double _d = 0) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm), d(_d) {
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
  GaussRandomSampler(Environment* _env, Stat_Class& _Stats,
		     CollisionDetection* _cd, CDInfo& _cdInfo,
		     DistanceMetric* _dm, string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm)    
    {
      //create option description
      po::options_description options("GaussRandom Options");
      options.add_options()
	("d", po::value<double>(&d)->default_value(0), "gaussian distribution distance")
	;

      //this is a hack here to prep args for commandline parser, fix later
      istringstream iss(params);
      int _argc = 0;
      char* _argv[10];
      string s;
      _argv[_argc] = (char*)malloc(sizeof(char)*(strlen(name())+1));
      strcpy(_argv[_argc], name());
      _argc++;
      while((iss >> s) && (_argc < 10)) {
	_argv[_argc] = (char*)malloc(sizeof(char)*(s.size()+1));
	strcpy(_argv[_argc], s.c_str());
	_argc++;
      }

      //store the parsed options into a variables_map     
      po::variables_map vm;
      po::store(po::parse_command_line(_argc, _argv, options), vm);
      po::notify(vm);
        
      //clean up memory
      for(int i=0; i<_argc; ++i)
        free(_argv[i]);

      if(d == 0)
	d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
    }
  ~GaussRandomSampler() {}

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
  
  bool operator()(const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)
    {
      string callee(name());
      callee += "::operator()";
            
      bool generated = false;
      int attempts = 0;
	
      CFG cfg1 = cfg_in;
      bool cfg1_free = cfg1.InBoundingBox(env) && 
	!cfg1.isCollision(env, Stats, cd, cdInfo, true, &callee);
      do {
	Stats.IncNodes_Attempted();
	attempts++;
	  
	CFG incr;
	incr.GetRandomRay(fabs(GaussianDistribution(fabs(d), fabs(d))), 
			      env, dm);
	CFG cfg2;
	cfg2.add(cfg1, incr);
	bool cfg2_free = cfg2.InBoundingBox(env) && 
	  !cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee);

	if(cfg1_free != cfg2_free) {
	  Stats.IncNodes_Generated();
	  generated = true;	
	  if(cfg1_free == KEEP_FREE)
	    cfg_out.push_back(cfg1);
	  else
	    cfg_out.push_back(cfg2);
	}
      } while (!generated && (attempts < max_attempts));
	
      return generated;
    }
};


template <typename CFG>
class BridgeTestRandomFreeSampler : public BaseSampler<CFG>
{
 private:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;
  DistanceMetric* dm;
  double d;

 public:
  BridgeTestRandomFreeSampler(Environment* _env, Stat_Class& _Stats, 
			      CollisionDetection* _cd, CDInfo& _cdInfo, 
			      DistanceMetric* _dm, double _d = 0) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm), d(_d) {
    if(d == 0)
      d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
  }
  BridgeTestRandomFreeSampler(Environment* _env, Stat_Class& _Stats,
			      CollisionDetection* _cd, CDInfo& _cdInfo,
			      DistanceMetric* _dm, string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm)    
    {
      //create option description
      po::options_description options("BridgeTestRandomFree Options");
      options.add_options()
	("d", po::value<double>(&d)->default_value(0), "gaussian distribution distance")
	;

      //this is a hack here to prep args for commandline parser, fix later
      istringstream iss(params);
      int _argc = 0;
      char* _argv[10];
      string s;
      _argv[_argc] = (char*)malloc(sizeof(char)*(strlen(name())+1));
      strcpy(_argv[_argc], name());
      _argc++;
      while((iss >> s) && (_argc < 10)) {
	_argv[_argc] = (char*)malloc(sizeof(char)*(s.size()+1));
	strcpy(_argv[_argc], s.c_str());
	_argc++;
      }

      //store the parsed options into a variables_map     
      po::variables_map vm;
      po::store(po::parse_command_line(_argc, _argv, options), vm);
      po::notify(vm);
        
      //clean up memory
      for(int i=0; i<_argc; ++i)
        free(_argv[i]);

      if(d == 0)
	d = (env->GetMultiBody(env->GetRobotIndex()))->GetMaxAxisRange();
    }
  ~BridgeTestRandomFreeSampler() {}

  const char* name() const { return "BridgeTestRandomFreeSampler"; }

  void print(ostream& os) const
    {
      os << name()
	 << " (d = " << d << ")";
    }
  
  bool operator()(const CFG& cfg_in, vector<CFG>& cfg_out, int max_attempts)
    {
      string callee(name());
      callee += "::operator()";  
      
      bool generated = false;
      int attempts = 0;
      
      do {
	Stats.IncNodes_Attempted();
	attempts++;
	
	CFG tmp = cfg_in;
	if(tmp.InBoundingBox(env) && 
	   !tmp.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  CFG mid = cfg_in;
	  CFG incr;
	  incr.GetRandomRay(fabs(GaussianDistribution(d, d))/2, env, dm);
	  CFG cfg1;
	  cfg1.subtract(mid, incr);
	  if(!cfg1.InBoundingBox(env) || 
	     cfg1.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	    CFG cfg2;
	    cfg2.add(mid, incr);
	    if(!cfg2.InBoundingBox(env) || 
	       cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	      Stats.IncNodes_Generated();
	      generated = true;
	      cfg_out.push_back(cfg_in);
	    }
	  }
	} else {
	  CFG cfg1 = cfg_in;
	  CFG incr;
	  incr.GetRandomRay(fabs(GaussianDistribution(d, d)), env, dm);
	  CFG cfg2;
	  cfg2.add(cfg1, incr);
	  if(!cfg2.InBoundingBox(env) || 
	     cfg2.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	     CFG mid;
	     mid.WeightedSum(cfg1, cfg2, 0.5);
	     if(mid.InBoundingBox(env) && 
		!mid.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	       Stats.IncNodes_Generated();
	       generated = true;
	       cfg_out.push_back(mid);
	     }
	  }
	}
      } while (!generated && (attempts < max_attempts));

      return generated;
    }
};

#endif
