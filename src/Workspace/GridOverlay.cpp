#include "GridOverlay.h"
#include "Utilities/MPUtils.h"
#include "Environment/BoundingBox.h"

GridOverlay::
GridOverlay(shared_ptr<Boundary> _b, int _length):
  m_length(_length), m_boundary(_b) {}

size_t 
GridOverlay::
LocateCell(const Cfg& _v) const {
  Point3d coor = _v.GetPoint();
  return IndexOfCoor(coor);
}


size_t 
GridOverlay::
IndexOfCoor(const Vector3d& _coor) const {
  
  const int dem = 3;
  int index = 0;
  int factor = 1;
  int temp;
  for(int i = 0; i < dem; ++i) {
    auto range = m_boundary->GetRange(i);
    temp = (int) m_length* (_coor[i] - range.first / range.second - range.first);

    index += temp * factor;
    factor *= m_length;
  }
  return index;
}
