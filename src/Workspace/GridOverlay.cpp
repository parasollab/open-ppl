#include "GridOverlay.h"
#include "Utilities/MPUtils.h"
#include <cmath>
#include "Environment/BoundingBox.h"

/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(shared_ptr<Boundary> _b, double _length):
    m_length(_length), m_boundary(_b) { }

/*------------------------------- Cell Finding -------------------------------*/

size_t
GridOverlay::
LocateCell(const Cfg& _v) const {
  return LocateCell(_v.GetPoint());
}

size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  int x, y, z, height, width, depth;

  auto range = m_boundary->GetRange(0);
  width = ceil((range.second - range.first) / m_length);
  x = (int)(width * ((_p[0] - range.first) / (range.second - range.first)));
 
  // for edge case of then the point in on the upper edge of the grid
  x = min(x, width - 1);

  range = m_boundary->GetRange(1);
  height = ceil((range.second - range.first) / m_length);
  y = (int)(height * ((_p[1] - range.first) / (range.second - range.first)));

  // for edge case of then the point in on the upper edge of the grid
  y = min(y, width - 1);

  range = m_boundary->GetRange(2);
  depth = ceil((range.second - range.first) / m_length);
  z = (int)(depth * ((_p[2] - range.first) / (range.second - range.first)));

  // for edge case of then the point in on the upper edge of the grid
  z = min(z, width - 1);

  return (width * height) * z + height * y + x;
}

/*----------------------------------------------------------------------------*/
