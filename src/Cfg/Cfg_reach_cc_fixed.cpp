#include "Cfg_reach_cc_fixed.h"
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
Cfg_reach_cc_fixed::
GetRandomCfg_CenterOfMass(Environment* env) {
  for(int i=0; i<6; ++i)
    v[i] = 0;

  link_tree->ResetTree();
  link_tree->RecursiveSample(0, true, gamma);
  link_lengths.clear();
  link_orientations.clear();
  link_tree->ExportTreeLinkLength(link_lengths, link_orientations);

  StoreData();
  
  obst = -1;
  tag = -1;
  clearance = -1;
}

void 
Cfg_reach_cc_fixed::
GetRandomRay(double incr, Environment* env, DistanceMetric* dm) {
  cerr << "Warning GetRandomRay not implemented yet\n";
}

Cfg* 
Cfg_reach_cc_fixed::
CreateNewCfg() const {
  Cfg* tmp = new Cfg_reach_cc_fixed(*this);
  return tmp;
}

Cfg* 
Cfg_reach_cc_fixed::
CreateNewCfg(vector<double>& _v) const {
  if(_v.size() < dof) {
    cout << "\n\nERROR in Cfg_reach_cc_fixed::CreateNewCfg(vector<double>), ";
    cout << "size of vector is less than dof\n";
    exit(-1);
  }
  vector<double> _data(_v.begin(), _v.begin()+dof);
  Cfg* tmp = new Cfg_reach_cc_fixed(_data);
  return tmp;
}
