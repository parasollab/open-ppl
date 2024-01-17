#include "ScaledEuclideanDistance.h"

/*------------------------------- Construction -------------------------------*/

ScaledEuclideanDistance::
ScaledEuclideanDistance() : EuclideanDistance() {
  this->SetName("ScaledEuclidean");
}


ScaledEuclideanDistance::
ScaledEuclideanDistance(XMLNode& _node) : DistanceMetricMethod(_node) {
  this->SetName("ScaledEuclidean");
  m_scale = _node.Read("scale", false, m_scale, 0.0, 1.0, "scale factor");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
ScaledEuclideanDistance::
Print(ostream& _os) const {
  EuclideanDistance::Print(_os);
  _os << "\tscale = " << m_scale << endl;
}

/*----------------------------- Distance Interface ---------------------------*/

double
ScaledEuclideanDistance::
Distance(const CfgType& _c1, const CfgType& _c2) {
  const CfgType c = _c2 - _c1;
  
  return pow(m_scale * this->PositionDistance(c)
      + (1 - m_scale) * this->OrientationDistance(c), this->m_r3);
}
