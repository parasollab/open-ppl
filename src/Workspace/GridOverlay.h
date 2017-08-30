#ifndef GRID_OVERLAY_H_
#define GRID_OVERLAY_H_

#include "ConfigurationSpace/Cfg.h"
#include "Vector.h"

class Boundary;
class WorkspaceDecomposition;
class WorkspaceRegion;


///////////////////////////////////////////////////////////////////////////////
/// A 3d grid overlay of a given boundary.
///
/// The cells are implicitly represented as either a tuple (x,y,z) or a single
/// 'cell index', which enumerates all cells with a single number. The grid is
/// defined over the minimum and maximum ranges of the boundary, so there may be
/// grid cells partially or completely outside of the boundary (depending its
/// shape and size).
///////////////////////////////////////////////////////////////////////////////
class GridOverlay {

  public:

    ///@name Local Types
    ///@{

    /// A mapping from a grid cell index to a set of workspace regions.
    typedef std::vector<std::vector<const WorkspaceRegion*>> DecompositionMap;

    ///@}
    ///@name Construction
    ///@{

    /// Construct a grid overlay with cells of a given length.
    /// @param _b The boundary to overlay.
    /// @param _length The cell length to use.
    GridOverlay(const Boundary* const _b, const double _length);

    ///@}
    ///@name Cell Finders
    ///@{

    /// Get the number of cells in the grid.
    size_t Size() const noexcept;

    /// Get the number of cells in a given dimension.
    /// @param _i The index for the dimension of interest.
    size_t Size(const size_t _i) const noexcept;

    /// Find the cell that contains the reference point of a configuration.
    /// @param _cfg The configuration to locate.
    size_t LocateCell(const Cfg& _cfg) const;

    /// Find the cell that contains a reference point.
    size_t LocateCell(const Point3d& _p) const;

    /// Find the cells that contain a given boundary's bounding box.
    /// @param _b The boundary of interest.
    /// @return A set of cell indexes that contain _b's bounding box.
    std::vector<size_t> LocateBBXCells(const Boundary* const _b) const;

    /// Find the cells that contain a bounding box around two points.
    /// @param _min The low-range values.
    /// @param _max The high-range values.
    /// @return A set of cell indexes that contain the bounding box around _min,
    ///         _max.
    std::vector<size_t> LocateBBXCells(const Point3d& _min, const Point3d& _max)
        const;

    ///@}
    ///@name Decomposition Mapping
    ///@{

    /// Create a map from grid cell index to the set of decomposition regions
    /// whos BBXs' touch the grid cell.
    /// @param _decomposition The workspace decomposition object to map.
    /// @return A mapping m, where m[i] gives the set of _decomposition regions
    ///         whos BBXs' touch grid cell i.
    DecompositionMap ComputeDecompositionMap(
        const WorkspaceDecomposition* const _decomposition) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get the triple of x,y,z indexes that define the cell containing a point.
    /// @param _p The point of interest.
    /// @return The x,y,z indexes of the cell holding _p.
    std::array<size_t, 3> Cell(const Point3d& _p) const noexcept;

    /// Get the cell index from x,y,z indexes.
    size_t CellIndex(const size_t _x, const size_t _y, const size_t _z) const
      noexcept;

    /// @overload
    size_t CellIndex(const array<size_t, 3>& _indexes) const noexcept;

    /// Get the z-index of a cell index.
    size_t ZIndex(const size_t _index) const noexcept;

    /// Get the y-index of a cell index.
    size_t YIndex(const size_t _index) const noexcept;

    /// Get the x-index of a cell index.
    size_t XIndex(const size_t _index) const noexcept;

    ///@}
    ///@name Internal State
    ///@{

    const Boundary* const m_boundary; ///< The boundary of the grid.

    const double m_length;            ///< Length of a cell.
    size_t m_num[3];                  ///< The number of cells in each dimension.

    static constexpr bool m_debug{false}; ///< Enable debugging messages?

    ///@}
};

#endif
