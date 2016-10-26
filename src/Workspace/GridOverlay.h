#ifndef GRID_OVERLAY_H_
#define GRID_OVERLAY_H_

#include "Cfg/Cfg.h"
#include "Environment/Boundary.h"
#include "Vector.h"

///////////////////////////////////////////////////////////////////////////////
/// A grid overlay of a given boundary.
///
/// The cells are implicitly represented as either a tuple (x,y,z) or a single
/// 'cell index', which enumerates all cells with a single number.
///////////////////////////////////////////////////////////////////////////////
class GridOverlay {

  public:

    ///@name Construction
    ///@{

    GridOverlay(shared_ptr<Boundary> _b, double _length);

    ///@}
    ///@name Cell Finders
    ///@{

    size_t Size() const noexcept; ///< Get the number of cells in the grid.
    size_t Size(const size_t _i) const noexcept; ///< Get num cells in an index.

    ////////////////////////////////////////////////////////////////////////////
    /// Find the cell that contains the reference point of a configuration.
    size_t LocateCell(const Cfg& _v) const;

    ////////////////////////////////////////////////////////////////////////////
    /// Find the cell that contains a reference point.
    size_t LocateCell(const Point3d& _p) const;

    ////////////////////////////////////////////////////////////////////////////
    /// Find the cells that contain a given boundary's bounding box.
    /// @param _b The boundary of interest.
    /// @return A set of cell indexes that contain _b's bounding box.
    vector<size_t> LocateBBXCells(const Boundary* _b) const;

    ////////////////////////////////////////////////////////////////////////////
    /// Find the cells that contain a bounding box around two points.
    /// @param _min The low-range values.
    /// @param _max The high-range values.
    /// @return A set of cell indexes that contain the bounding box around _min,
    ///         _max.
    vector<size_t> LocateBBXCells(const Point3d& _min, const Point3d& _max) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// Get the triple of x,y,z indexes that define the cell containing a point.
    /// @param _p The point of interest.
    /// @return The x,y,z indexes of the cell holding _p.
    array<size_t, 3> Cell(const Point3d& _p) const noexcept;

    ////////////////////////////////////////////////////////////////////////////
    /// Get the cell index from x,y,z indexes.
    size_t CellIndex(const size_t _x, const size_t _y, const size_t _z) const
      noexcept;
    size_t CellIndex(const array<size_t, 3>& _indexes) const noexcept;

    ////////////////////////////////////////////////////////////////////////////
    /// Get the z-index of a cell index.
    size_t ZIndex(const size_t _index) const noexcept;

    ////////////////////////////////////////////////////////////////////////////
    /// Get the y-index of a cell index.
    size_t YIndex(const size_t _index) const noexcept;

    ////////////////////////////////////////////////////////////////////////////
    /// Get the x-index of a cell index.
    size_t XIndex(const size_t _index) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    shared_ptr<Boundary> m_boundary; ///< The boundary of the grid.

    double m_length;                 ///< Length of a cell.
    size_t m_num[3];                 ///< The number of cells in each dimension.

    ///@}
};

/*---------------------------- Inlined Helpers -------------------------------*/

inline
size_t
GridOverlay::
CellIndex(const size_t _x, const size_t _y, const size_t _z) const noexcept {
  return (m_num[0] * m_num[1]) * _z + m_num[0] * _y + _x;
}


inline
size_t
GridOverlay::
CellIndex(const array<size_t, 3>& _indexes) const noexcept {
  return CellIndex(_indexes[0], _indexes[1], _indexes[2]);
}


inline
size_t
GridOverlay::
ZIndex(const size_t _index) const noexcept {
  // To find the z-index, we see how many x-y slices we can remove.
  return _index / (m_num[0] * m_num[1]);
}


inline
size_t
GridOverlay::
YIndex(const size_t _index) const noexcept {
  // To find the y-index, we first remove the z components and then see how many
  // x rows we can remove.
  return (_index % (m_num[0] * m_num[1])) / m_num[0];
}


inline
size_t
GridOverlay::
XIndex(const size_t _index) const noexcept {
  // To find the x-index, we remove the z and y components.
  return _index % (m_num[0] * m_num[1]) % m_num[0];
}

/*----------------------------------------------------------------------------*/

#endif
