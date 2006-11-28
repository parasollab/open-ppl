#ifndef MedialAxisSamplers_h
#define MedialAxisSamplers_h

#include "BaseSampler.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

#include "my_program_options.hpp"
#include <sstream>

template <typename CFG>
class FreeMedialAxisSampler : public BaseSampler<CFG>
{
 public:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;
  DistanceMetric* dm;
  int clearance, penetration;

  FreeMedialAxisSampler(Environment* _env, Stat_Class& _Stats, 
			CollisionDetection* _cd, CDInfo& _cdInfo, 
			DistanceMetric* _dm, int _c, int _p) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm),
    clearance(_c), penetration(_p) {} 
  FreeMedialAxisSampler(Environment* _env, Stat_Class& _Stats,
			CollisionDetection* _cd, CDInfo& _cdInfo,
			DistanceMetric* _dm, string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo), dm(_dm)    
    {
      //create option description
      po::options_description options("MedialAxis Options");
      options.add_options()
	("clearance,c", po::value<int>(&clearance)->default_value(6), "number of clearance rays")
	("penetration,p", po::value<int>(&penetration)->default_value(6), "number of penetration rays")
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
    }
  ~FreeMedialAxisSampler() {}

  const char* name() const { return "FreeMedialAxisSampler"; }

  void print(ostream& os) const
    {
      os << name() 
	 << " (clearance = " << clearance 
	 << ", penetration = " << penetration << ")";
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
	tmp.PushToMedialAxis(env, Stats, cd, cdInfo, dm, clearance, penetration);
	if(tmp.InBoundingBox(env) && !tmp.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  Stats.IncNodes_Generated();
	  generated = true;
	  cfg_out.push_back(tmp);
	}
      } while (!generated && (attempts < max_attempts));

      return generated;
    }
};

#endif
