#include "GridOverlay.h"
#include "Utilities/MPUtils.h"
#include <cmath>
#include "Environment/BoundingBox.h"

/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(shared_ptr<Boundary> _b, double _length):
    m_boundary(_b),  m_length(_length) {
  // Compute the number of cells in each dimension.
  for(size_t i = 0; i < 3; ++i) {
    auto range = m_boundary->GetRange(i);
    m_num[i] = ceil((range.second - range.first) / m_length);
  }
}

/*------------------------------- Cell Finding -------------------------------*/

size_t
GridOverlay::
Size() const noexcept {
  return m_num[0] * m_num[1] * m_num[2];
}


size_t
GridOverlay::
Size(const size_t _i) const noexcept {
  return m_num[_i];
}


size_t
GridOverlay::
LocateCell(const Cfg& _v) const {
  return LocateCell(_v.GetPoint());
}


size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  return CellIndex(Cell(_p));
}


vector<size_t>
GridOverlay::
LocateBBXCells(const Boundary* _b) const {
  // Find the bouding points for _b's bounding box.
  Point3d min, max;
  for(size_t i = 0; i < 3; ++i) {
    auto range = _b->GetRange(i);
    min[i] = range.first;
    max[i] = range.second;
  }
  return LocateBBXCells(min, max);
}


vector<size_t>
GridOverlay::
LocateBBXCells(const Point3d& _min, const Point3d& _max) const {
  auto min = Cell(_min);
  auto max = Cell(_max);

  // Create space for the appropriate number of cells.
  vector<size_t> output;
  output.reserve((max[0] - min[0] + 1) *
                 (max[1] - min[1] + 1) *
                 (max[2] - min[2] + 1));

  // Populate the cell list.
  for(size_t z = min[2]; z <= max[2]; ++z) {
    for(size_t y = min[1]; y <= max[1]; ++y) {
      size_t first = CellIndex(min[0], y, z);
      size_t last  = CellIndex(max[0], y, z);
      for(size_t i = first; i <= last; ++i)
        output.push_back(i);
    }
  }

  return output;
}

/*------------------------------- Helpers ------------------------------------*/

array<size_t, 3>
GridOverlay::
Cell(const Point3d& _p) const noexcept {
  array<size_t, 3> cell;

  for(size_t i = 0; i < 3; ++i) {
    auto range = m_boundary->GetRange(i);
    cell[i] = m_num[i] * (_p[i] - range.first) / (range.second - range.first);
    // Catch edge-case for cells on the maximal boundary.
    cell[i] = min(cell[i], m_num[i] - 1);
  }

  return cell;
}

/*----------------------------------------------------------------------------*/
