#include "Samplers/SimilarStructureSampler.h"

ostream& operator<<(ostream& _os, const DistributionConstant& _d) {
  _os << "constant(" << _d.m_value << ")";
  return _os;
}

ostream& operator<<(ostream& _os, const DistributionUniform& _d) {
  _os << "uniform(" << _d.m_low << ", " << _d.m_high << ")";
  return _os;
}

ostream& operator<<(ostream& _os, const DistributionGaussian& _d) {
  _os << "gaussian(" << _d.m_mean << ", " << _d.m_std << ")";
  return _os;
}

