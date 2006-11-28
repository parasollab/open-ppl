#ifndef UniformSampler_h
#define UniformSampler_h

#include "BaseSampler.h"
class Environment;
class Stat_Class;
class CollisionDetection;
class CDInfo;

#include "my_program_options.hpp"
#include <sstream>


template <typename CFG>
class UniformRandomSampler : public BaseSampler<CFG>
{
 private:
  Environment* env;
  Stat_Class& Stats;

 public:
  UniformRandomSampler(Environment* _env, Stat_Class& _Stats) :
    env(_env), Stats(_Stats) {} 
  UniformRandomSampler(Environment* _env, Stat_Class& _Stats,
		       string params) :
    env(_env), Stats(_Stats)
    {
      //create option description
      po::options_description options("UniformRandom Options");

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
  ~UniformRandomSampler() {}

  const char* name() const { return "UniformRandomSampler"; }

  void print(ostream& os) const
    {
      os << name();
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
	
	CFG tmp;
	tmp.GetRandomCfg(env);
	if(tmp.InBoundingBox(env)) {
	  Stats.IncNodes_Generated();
	  generated = true;
	  cfg_out.push_back(tmp);
	}
      } while (!generated && (attempts < max_attempts));
      
      return generated;
    }
};


template <typename CFG>
class UniformRandomFreeSampler : public BaseSampler<CFG>
{
 private:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;

 public:
  UniformRandomFreeSampler(Environment* _env, Stat_Class& _Stats, 
			   CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  UniformRandomFreeSampler(Environment* _env, Stat_Class& _Stats,
			   CollisionDetection* _cd, CDInfo& _cdInfo,
			   string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo)
    {
      //create option description
      po::options_description options("UniformRandomFree Options");

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
  ~UniformRandomFreeSampler() {}

  const char* name() const { return "UniformRandomFreeSampler"; }

  void print(ostream& os) const
    {
      os << name();
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
	  
	CFG tmp;
	tmp.GetRandomCfg(env);
	if(tmp.InBoundingBox(env) && !tmp.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  Stats.IncNodes_Generated();
	  generated = true;
	  cfg_out.push_back(tmp);
	}
      } while (!generated && (attempts < max_attempts));

      return generated;
    }
};


template <typename CFG>
class UniformRandomCollisionSampler : public BaseSampler<CFG>
{
 private:
  Environment* env;
  Stat_Class& Stats;
  CollisionDetection* cd;
  CDInfo& cdInfo;

 public:
  UniformRandomCollisionSampler(Environment* _env, Stat_Class& _Stats, 
				CollisionDetection* _cd, CDInfo& _cdInfo) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo) {} 
  UniformRandomCollisionSampler(Environment* _env, Stat_Class& _Stats,
				CollisionDetection* _cd, CDInfo& _cdInfo,
				string params) :
    env(_env), Stats(_Stats), cd(_cd), cdInfo(_cdInfo)
    {
      //create option description
      po::options_description options("UniformRandomCollision Options");

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
  ~UniformRandomCollisionSampler() {}

  const char* name() const { return "UniformRandomCollisionSampler"; }

  void print(ostream& os) const
    {
      os << name();
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
	  
	CFG tmp;
	tmp.GetRandomCfg(env);
	if(tmp.InBoundingBox(env) && tmp.isCollision(env, Stats, cd, cdInfo, true, &callee)) {
	  Stats.IncNodes_Generated();
	  generated = true;
	  cfg_out.push_back(tmp);
	}
      } while (!generated && (attempts < max_attempts));

      return generated;
    }
};


/**********



template <typename Region, typename Sampler>
class RegionSampler 
{
 public:
  Region region;
  Sampler internal_sampler;
  int max_attempts;

  RegionSampler(const Region& r, const Sampler& s, int max = 100) :
    region(r), internal_sampler(s), max_attempts(max) {}
  ~RegionSampler() {}

  template <typename InputIterator, typename OutputIterator>
    OutputIterator operator()(InputIterator first, InputIterator last,
			      OutputIterator result, bool exact = false) 
    {
      typedef typename iterator_traits<InputIterator>::value_type CFG;

      while(first != last) {
	int attempts = 0;
	bool generated = true;
	InputIterator next = first+1;	  
	do {
	  attempts++;
	  vector<CFG> tmp_out;
	  internal_sampler(first, next, 
			   back_insert_iterator<vector<CFG> >(tmp_out),
			   exact);	  
	  for(vector<CFG>::const_iterator I = tmp_out.begin();
	      I != tmp_out.end(); ++I) {
	    if(region.in_region(*I)) {
	      generated = true;
	      *result = *I;
	      result++;
	    }
	  }
	} while (exact && !generated && (attempts < max_attempts));
	first++;
      } 
      return result;
    }
};

class RegionSampler<SphereRegion, UniformRandomSampler> 
{
 public:
  SphereRegion region;
  UniformSampler internal_sampler;
  int max_attempts;

  RegionSampler(const SphereRegion& r, const UniformSampler& s, int max = 100) :
    region(r), internal_sampler(s), max_attempts(max) {}
  ~RegionSampler() {}

  template <typename InputIterator, typename OutputIterator>
    OutputIterator operator()(InputIterator first, InputIterator last,
			      OutputIterator result, bool exact = false) 
    {
      typedef typename iterator_traits<InputIterator>::value_type CFG;

      while(first != last) {
	int attempts = 0;
	bool generated = true;
	do {
	  attempts++;
	  CFG tmp;
	  tmp.GetRandomRay(drand48()*region.radius/1.1, env, dm);
	  tmp.add(region.center, tmp);
	  if(tmp.InBoundingBox(env) && region.in_region(tmp)) {
	    generated = true;
	    *result = tmp;
	    result++;
	  }
	} while (exact && !generated && (attempts < max_attempts));
	first++;
      } 
      return result;
    }
};



template <typename CFG>
class SphereRegion
{
 public:
  CFG center;
  double radius;
  Environment* env;
  DistanceMetric* dm;

  SphereRegion(const CFG& c, double d, const Environment* e,
	       const DistanceMetric* _dm) :
    center(c), radius(d), env(e), dm(_dm) {}
  ~SphereRegion() {}

  bool in_region(const CFG& c) const 
    {
      return (dm->Distance(env, center, c) <= radius);
    }

};

**********/



#endif
