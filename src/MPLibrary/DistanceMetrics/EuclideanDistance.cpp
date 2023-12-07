#include "EuclideanDistance.h"

/*------------------------------- Construction -------------------------------*/

EuclideanDistance::
EuclideanDistance(bool _normalize) :
    MinkowskiDistance(2, 2, 1. / 2, _normalize) {
  this->SetName("Euclidean");
}


EuclideanDistance::
EuclideanDistance(XMLNode& _node) : DistanceMetricMethod(_node), 
                                    MinkowskiDistance(_node) {
  this->SetName("Euclidean");

  this->m_r1 = 2;
  this->m_r2 = 2;
  this->m_r3 = 1.0/2;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/
