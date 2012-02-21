#include "ScaledEuclideanDistance.h"
#include "Cfg_reach_cc.h"


ScaledEuclideanDistance::ScaledEuclideanDistance() : EuclideanDistance(), m_sValue(0.5){
  m_name = "scaledEuclidean";
}

ScaledEuclideanDistance::
ScaledEuclideanDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warn) : EuclideanDistance(_node, _problem, false){
  m_name = "scaledEuclidean";
  m_sValue = _node.numberXMLParameter("scale", false, 0.5, 0.0, 1.0, "Scale Factor");
  if(_warn)
    _node.warnUnrequestedAttributes();
}

ScaledEuclideanDistance::~ScaledEuclideanDistance() {
}

double ScaledEuclideanDistance::GetS() const { 
  return m_sValue; 
}

bool ScaledEuclideanDistance::operator==(const ScaledEuclideanDistance& _dm) const {
  if(GetName() != _dm.GetName()) {
    return false;
  } else {
    return ((m_sValue-_dm.GetS() < 0.000000001) && (m_sValue-_dm.GetS() > -0.000000001));
  }
}

void ScaledEuclideanDistance::PrintOptions(ostream& _os) const {
  _os << "    " << GetName() << "::  ";
  _os << "scale = " << m_sValue;
  _os << endl;
}

double ScaledEuclideanDistance::Distance(Environment* _env, const Cfg& _c1, const Cfg& _c2) {
  double dist;
  dist = ScaledDistance<CfgType>(_env, _c1, _c2, m_sValue);
  return dist;
}
