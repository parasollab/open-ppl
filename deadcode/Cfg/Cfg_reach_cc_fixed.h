#ifndef Cfg_reach_cc_fixed_h
#define Cfg_reach_cc_fixed_h

#include "Cfg_reach_cc.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ReachableCfgs
/// @ingroup DeadCode
/// @brief TODO Dead Code
///
/// TODO
/// @todo Dead code. Figure out what to do with this.
////////////////////////////////////////////////////////////////////////////////
class Cfg_reach_cc_fixed : public Cfg_reach_cc {
 public:
  Cfg_reach_cc_fixed();
  Cfg_reach_cc_fixed(const Cfg&c);
  virtual ~Cfg_reach_cc_fixed();

  virtual const string GetName() const { return "Cfg_reach_cc_fixed"; }
  static size_t GetNumOfJoints() { return m_dof; }

  virtual void GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm=true);

  virtual ostream& print_base(ostream& os) const { return os; }

 protected:
};

#endif
