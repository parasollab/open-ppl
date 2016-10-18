#include "GridOverlay.h"
#include "Utilities/MPUtils.h"
#include "Environment/BoundingBox.h"

/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(shared_ptr<Boundary> _b, double _length):
    m_length(_length), m_boundary(_b) { }


size_t
GridOverlay::
LocateCell(const Cfg& _v) const {
  return LocateCell(_v.GetPoint());
}

/*------------------------------- Cell Finding -------------------------------*/

size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  const int dimension = 3;
  int index = 0;
  int factor = 1;
  int temp;
  for(int i = 0; i < dimension; ++i) {
    auto range = m_boundary->GetRange(i);
    temp = (int)m_length * ((_p[i] - range.first) / (range.second - range.first));

    index += temp * factor;
    factor *= m_length;
  }
  return index;
}

/*----------------------------------------------------------------------------*/
