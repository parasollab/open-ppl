#ifndef ObstacleBasedSamplers_h
#define ObstacleBasedSamplers_h

#include "BaseSampler.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

#include "my_program_options.hpp"
#include <sstream>

template <typename CFG>
class ObstacleBasedSampler : public BaseSampler<CFG>
{
 private:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;
  DistanceMetric* dm;
  int n_shells_free, n_shells_coll;
  double step_size;

 public:
  ObstacleBasedSampler(Environment* _env, Stat_Class& _Stats, 
		       CollisionDetection* _cd, CDInfo& _cdInfo,
		       DistanceMetric* _dm, int _free = 1, int _coll = 0, 
		       double _step = 0) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm), 
    n_shells_free(_free), n_shells_coll(_coll), step_size(_step) {
    if(step_size <= 0)
      step_size = min(env->GetPositionRes(), env->GetOrientationRes());
  }
  ObstacleBasedSampler(Environment* _env, Stat_Class& _Stats,
		       CollisionDetection* _cd, CDInfo& _cdInfo,
		       DistanceMetric* _dm, string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm)    
    {
      //create option description
      po::options_description options("ObstacleBased Options");
      options.add_options()
	("free-shells,f", po::value<int>(&n_shells_free)->default_value(1), "number of free shells")
	("coll-shells,c", po::value<int>(&n_shells_coll)->default_value(0), "number of collision shells")
	("stepsize,s", po::value<double>(&step_size)->default_value(0), "obstacle boundary search stepsize")
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

      if(step_size <= 0)
	step_size = min(env->GetPositionRes(), env->GetOrientationRes());
    }
  ~ObstacleBasedSampler() {}

  const char* name() const { return "ObstacleBasedSampler"; }

  void print(ostream& os) const
    {
      os << name() 
	 << " (n_shells_free = " << n_shells_free 
	 << ", n_shells_coll = " << n_shells_coll 
	 << ", stepsize = " << step_size << ")";
    }
  
  template <typename OutputIterator>
    OutputIterator GenerateShells(CFG c_free, CFG c_coll, CFG incr, 
				  OutputIterator result) 
    {
      string callee(name());
      callee += "::GenerateShells";
      for(int i=0; i<n_shells_free; ++i) {
	if(c_free.InBoundingBox(env) && 
	   !c_free.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  Stats.IncNodes_Generated();
	  *result = c_free;
	  result++;
	}
	c_free.Increment(incr);
      }
      
      CFG tmp;
      incr.subtract(tmp, incr);
      for(int i=0; i<n_shells_coll; ++i) {
	if(c_coll.InBoundingBox(env) && 
	   c_coll.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  Stats.IncNodes_Generated();
	  *result = c_coll;
	  result++;
	}
	c_coll.Increment(incr);
      }
      
      return result;
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

	CFG c1 = cfg_in;
	bool c1_bbox = c1.InBoundingBox(env);
	bool c1_free = !c1.isCollision(env, Stats, cd, cdInfo, true, &callee);

	CFG c2 = c1;
	bool c2_bbox = c1_bbox;
	bool c2_free = c1_free;

	CFG r;
	r.GetRandomRay(step_size, env, dm);
	while(c1_bbox && c2_bbox && (c1_free == c2_free)) {
	  c1 = c2;
	  c1_bbox = c2_bbox;
	  c1_free = c2_free;
	 
	  c2.Increment(r);
	  c2_bbox = c2.InBoundingBox(env);
	  c2_free = !c2.isCollision(env, Stats, cd, cdInfo, true, &callee);
	}

	if(c1_bbox && c2_bbox) {
	  generated = true;
	  if(c1_free) {
	    CFG tmp;
	    r.subtract(tmp, r);
	    GenerateShells(c1, c2, r, 
			   back_insert_iterator<vector<CFG> >(cfg_out));
	  } else 
	    GenerateShells(c2, c1, r,
			   back_insert_iterator<vector<CFG> >(cfg_out));
	}
      } while (!generated && (attempts < max_attempts));
      
      return generated;
    }
};

#endif
