#include "MinkowskiDistance.h"


MinkowskiDistance::MinkowskiDistance() : DistanceMetricMethod() {
  m_name = "minkowski";
  m_r1 = 3;
  m_r2 = 3;
  m_r3 = 1.0/3;
}

MinkowskiDistance::
MinkowskiDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : DistanceMetricMethod(_node, _problem, false) {
  m_name = "minkowski";
  m_r1 = _node.numberXMLParameter("r1", false, 3.0, 0.0, 1000.0, "r1");
  m_r2 = _node.numberXMLParameter("r2", false, 3.0, 0.0, 1000.0, "r2");
  m_r3 = _node.numberXMLParameter("r3", false, 1.0/3.0, 0.0, 1000.0, "r3");
  if(_warn)
    _node.warnUnrequestedAttributes();
}

MinkowskiDistance::~MinkowskiDistance() {}

double MinkowskiDistance::GetR1() const { 
  return m_r1; 
}

double MinkowskiDistance::GetR2() const { 
  return m_r2; 
}

double MinkowskiDistance::GetR3() const { 
  return m_r3; 
}

bool MinkowskiDistance::operator==(const MinkowskiDistance& _dm) const {
  if(GetName() != _dm.GetName()) {
    return false;
  } else {
    return ( ((m_r1-_dm.GetR1() < 0.000000001) && (m_r1-_dm.GetR1() > -0.000000001)) &&
             ((m_r2-_dm.GetR2() < 0.000000001) && (m_r2-_dm.GetR2() > -0.000000001)) &&
             ((m_r3-_dm.GetR3() < 0.000000001) && (m_r3-_dm.GetR3() > -0.000000001)) );
  }
}

void MinkowskiDistance::PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << ":: ";
  _os << m_r1 << " " << m_r2 << " " << m_r3;
  _os << endl;
}

double MinkowskiDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist,pos=0,orient=0;
  Cfg *pC = _c1.CreateNewCfg();
  pC->subtract(_c1,_c2);
  vector<double> p = pC->GetPosition(); // position values
  vector<double> o = pC->GetOrientation(); //orientation values
  for(size_t i=0; i<p.size(); i++) {
    if(p[i] < 0) 
      p[i] = -p[i];
    pos += pow(p[i], m_r1);
  }
  for(size_t i=0;i<o.size();i++) {
    orient += pow(o[i], m_r2);
  }
  dist = pow(pos+orient, m_r3);
  delete pC;
  return dist;
}

