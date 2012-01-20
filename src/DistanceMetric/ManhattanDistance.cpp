#include "ManhattanDistance.h"



ManhattanDistance::ManhattanDistance() : DistanceMetricMethod() {
  m_name = "manhattan";
}

ManhattanDistance::
ManhattanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, _warn) {
  m_name = "manhattan";
}

ManhattanDistance::~ManhattanDistance() {
}

void ManhattanDistance::PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << "::  " << endl;
}

double ManhattanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist = 0;
  Cfg *pC = _c1.CreateNewCfg();
  vector<double> dt = pC->GetData(); // position values
  for(size_t i=0; i < dt.size(); i++) {
    if(dt[i] < 0) 
      dist = dist-dt[i];
    else
      dist += dt[i];
  }
  delete pC;
  return dist;
}
