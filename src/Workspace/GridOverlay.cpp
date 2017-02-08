#include <cmath>

#include "GridOverlay.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/MPUtils.h"

/*------------------------------- Construction -------------------------------*/

GridOverlay::
GridOverlay(const Boundary* const _b, const double _length):
    m_boundary(_b),  m_length(_length) {
  // Compute the number of cells in each dimension.
  for(size_t i = 0; i < 3; ++i)
    m_num[i] = std::ceil(m_boundary->GetRange(i).Length() / m_length);
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
LocateCell(const Cfg& _cfg) const {
  return LocateCell(_cfg.GetPoint());
}


size_t
GridOverlay::
LocateCell(const Point3d& _p) const {
  return CellIndex(Cell(_p));
}


std::vector<size_t>
GridOverlay::
LocateBBXCells(const Boundary* const _b) const {
  Point3d min, max;

  for(size_t i = 0; i < 3; ++i) {
    auto range = _b->GetRange(i);
    min[i] = range.min;
    max[i] = range.max;
  }

  return LocateBBXCells(min, max);
}


std::vector<size_t>
GridOverlay::
LocateBBXCells(const Point3d& _min, const Point3d& _max) const {
  const auto min = Cell(_min);
  const auto max = Cell(_max);

  // Create space for the appropriate number of cells.
  std::vector<size_t> output;
  output.reserve((max[0] - min[0] + 1) *
                 (max[1] - min[1] + 1) *
                 (max[2] - min[2] + 1));

  // Populate the cell list.
  for(size_t z = min[2]; z <= max[2]; ++z) {
    for(size_t y = min[1]; y <= max[1]; ++y) {
      const size_t first = CellIndex(min[0], y, z);
      const size_t last  = CellIndex(max[0], y, z);
      for(size_t i = first; i <= last; ++i)
        output.push_back(i);
    }
  }

  return output;
}

/*------------------------------- Helpers ------------------------------------*/

std::array<size_t, 3>
GridOverlay::
Cell(const Point3d& _p) const noexcept {
  std::array<size_t, 3> cell;

  for(size_t i = 0; i < 3; ++i) {
    const auto range = m_boundary->GetRange(i);
    cell[i] = m_num[i] * (_p[i] - range.min) / range.Length();
    // Catch edge-case for cells on the maximal boundary.
    cell[i] = std::min(cell[i], m_num[i] - 1);
  }

  return cell;
}


size_t
GridOverlay::
CellIndex(const size_t _x, const size_t _y, const size_t _z) const noexcept {
  return (m_num[0] * m_num[1]) * _z + m_num[0] * _y + _x;
}


size_t
GridOverlay::
CellIndex(const array<size_t, 3>& _indexes) const noexcept {
  return CellIndex(_indexes[0], _indexes[1], _indexes[2]);
}


size_t
GridOverlay::
ZIndex(const size_t _index) const noexcept {
  // To find the z-index, we see how many x-y slices we can remove.
  return _index / (m_num[0] * m_num[1]);
}


size_t
GridOverlay::
YIndex(const size_t _index) const noexcept {
  // To find the y-index, we first remove the z components and then see how many
  // x rows we can remove.
  return (_index % (m_num[0] * m_num[1])) / m_num[0];
}


size_t
GridOverlay::
XIndex(const size_t _index) const noexcept {
  // To find the x-index, we remove the z and y components.
  return _index % (m_num[0] * m_num[1]) % m_num[0];
}

/*----------------------------------------------------------------------------*/
