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
    Cfg_reach_cc(base, len, ori) {
    m_posDof = 0;
    m_dof = m_numOfJoints;
    m_v.resize(m_dof);
  }
  virtual ~Cfg_reach_cc_fixed();

  virtual const char* GetName() const { return "Cfg_reach_cc_fixed"; }

  virtual bool ConfigEnvironment(Environment* _env) const;

  virtual void GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm=true);

  virtual ostream& print_base(ostream& os) const { return os; }

 protected:
};

#endif
