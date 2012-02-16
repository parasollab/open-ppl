#include "Cfg_reach_cc_fixed.h"
#include "Environment.h"
#include <numeric>

#define TWO_PI 6.2831853072

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed() : Cfg_reach_cc() {
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(int _numofjoints) : Cfg_reach_cc(_numofjoints) {
   
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const Vector6D& _v) : Cfg_reach_cc(_v) {
  
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const vector<double>& _v) : Cfg_reach_cc(_v) {
  
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(const Cfg& c) : Cfg_reach_cc(c) {
   
}

Cfg_reach_cc_fixed::
Cfg_reach_cc_fixed(double x, double y, double z, 
	     double roll, double pitch, double yaw) 
  : Cfg_reach_cc(x, y, z, roll, pitch, yaw) {
    
}

Cfg_reach_cc_fixed::
~Cfg_reach_cc_fixed() {
}

void 
Cfg_reach_cc_fixed::
GetRandomCfg(double R, double rStep) {
  cerr << "Warning GetRandomCfg not implemented yet\n";
}

void 
Cfg_reach_cc_fixed::GetRandomCfg_CenterOfMass(Environment *_env) {
  GetRandomCfg_CenterOfMass(_env, _env->GetBoundingBox());
}

void 
Cfg_reach_cc_fixed::
GetRandomCfg_CenterOfMass(Environment* env, shared_ptr<Boundary> _bb) {
  for(int i=0; i<6; ++i)
    m_v[i] = 0;

  link_tree->ResetTree();
   if(is_closed_chain)
    link_tree->RecursiveSample(0, true, gamma);
  else
    link_tree->RecursiveSample(-1, true, gamma);
  link_lengths.clear();
  link_orientations.clear();
  link_tree->ExportTreeLinkLength(link_lengths, link_orientations);

  StoreData();
}

void 
Cfg_reach_cc_fixed::
GetRandomRay(double incr, Environment* env, shared_ptr<DistanceMetricMethod> dm, bool _norm) {
  cerr << "Warning GetRandomRay not implemented yet\n";
}

