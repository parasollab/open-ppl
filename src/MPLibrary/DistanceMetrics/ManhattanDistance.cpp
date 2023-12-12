#include "ManhattanDistance.h"

/*------------------------------- Construction -------------------------------*/

ManhattanDistance::
ManhattanDistance(bool _normalize) :
    MinkowskiDistance(1, 1, 1, _normalize) {
  this->SetName("Manhattan");
}


ManhattanDistance::
ManhattanDistance(XMLNode& _node) : DistanceMetricMethod(_node),
                                    MinkowskiDistance(_node) {
  this->SetName("Manhattan");

  this->m_r1 = 1;
  this->m_r2 = 1;
  this->m_r3 = 1;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/
