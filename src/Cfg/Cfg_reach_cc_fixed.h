#ifndef Cfg_reach_cc_fixed_h
#define Cfg_reach_cc_fixed_h

#include "Cfg_reach_cc.h"

class Cfg_reach_cc_fixed : public Cfg_reach_cc {
 public:
  Cfg_reach_cc_fixed();
  Cfg_reach_cc_fixed(int _numofjoints);
  Cfg_reach_cc_fixed(const Vector6D& _v);
  Cfg_reach_cc_fixed(const vector<double>& _v);
  Cfg_reach_cc_fixed(const Cfg&c);
  Cfg_reach_cc_fixed(double x, double y, double z, 
	       double roll, double pitch, double yaw);
  Cfg_reach_cc_fixed(const Vector6D& base,
		     const vector<double>& len, const vector<int>& ori) :
    Cfg_reach_cc(base, len, ori) {}
  virtual ~Cfg_reach_cc_fixed();

  virtual const char* GetName() const { return "Cfg_reach_cc_fixed"; }

  virtual void GetRandomCfg(double R, double rStep);
  virtual void GetRandomCfg(Environment* _env,shared_ptr<Boundary> _bb) {
    Cfg_reach_cc::GetRandomCfg(_env,_bb);
  }
  virtual void GetRandomCfg(Environment* _env){
    Cfg_reach_cc::GetRandomCfg(_env);
  }

  /*
  virtual void GetRandomCfg(Environment* env, DistanceMetric* _dm, 
                            double length);
  */
  virtual void GetRandomCfg_CenterOfMass(Environment* env);
  virtual void GetRandomCfg_CenterOfMass(Environment* _env,shared_ptr<Boundary> _bb);
  virtual void GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm=true);

 protected:
};

#endif
