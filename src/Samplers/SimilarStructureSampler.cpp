#include "Samplers/SimilarStructureSampler.h"

ostream&
operator<<(ostream& _os, const DistributionConstant& _d) {
  return _os << "constant(" << _d.m_value << ")";
}

ostream&
operator<<(ostream& _os, const DistributionUniform& _d) {
  return _os << "uniform(" << _d.m_low << ", " << _d.m_high << ")";
}

ostream&
operator<<(ostream& _os, const DistributionGaussian& _d) {
  return _os << "gaussian(" << _d.m_mean << ", " << _d.m_std << ")";
}

