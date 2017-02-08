#ifndef WORKSPACE_REGION_H_
#define WORKSPACE_REGION_H_

#include <memory>
#include <vector>

#include "Geometry/GMSPolygon.h"

class Boundary;
class WorkspaceDecomposition;


////////////////////////////////////////////////////////////////////////////////
/// A convex region of workspace represented as a triangulated mesh.
///
/// Since many regions may share the same points, this object stores only point
/// indexes. The actual points are stored by the owning WorkspaceDecomposition.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceRegion final {

  public:

    ///@name Local Types
    ///@{

    typedef GMSPolygon Facet; ///< A triangle facet.

    ///@}
    ///@name Construction
    ///@{

    /// @warning Default construction makes no sense for this object, but the
    ///          stapl graph requires it.
    WorkspaceRegion();

    /// Construct a workspace region attached to a specific decomposition
    /// object.
    /// @param _wd The owning workspace decomposition.
    WorkspaceRegion(WorkspaceDecomposition* const _wd);

    ///@}
    ///@name Modifiers
    ///@{

    void AddPoint(const size_t _i);
    void AddFacet(Facet&& _f);

    /// Add a boundary and assume ownership of it.
    /// @param _b The (dynamically allocated) boundary to add.
    void AddBoundary(Boundary* const _b);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the number of points in the region.
    const size_t GetNumPoints() const noexcept;

    /// Get the number of facets in the region.
    const size_t GetNumFacets() const noexcept;

    /// Get the _i'th point of this region.
    const Point3d& GetPoint(const size_t _i) const noexcept;

    /// Get a list of all points in this region.
    const std::vector<Point3d> GetPoints() const noexcept;

    /// Get the set of facets that border this region.
    const std::vector<Facet>& GetFacets() const noexcept;

    /// Get the boundary for this region.
    const Boundary* GetBoundary() const noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Check if a given point is part of this region's boundary.
    const bool HasPoint(const Point3d& _p) const;

    /// Find the center of this region.
    const Point3d FindCenter() const;

    /// Find shared points with another workspace region.
    const std::vector<Point3d> FindSharedPoints(const WorkspaceRegion& _wr) const;

    /// Find shared facets with another workspace region.
    const std::vector<const Facet*> FindSharedFacets(const WorkspaceRegion& _wr)
        const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    WorkspaceDecomposition* const m_decomposition; ///< The owning decomposition.

    std::vector<size_t> m_points;  ///< The bounding point indexes of this region.
    std::vector<Facet> m_facets;   ///< The bounding facets of this region.

    std::shared_ptr<Boundary> m_boundary; ///< The boundary for this region.

    ///@}
};

#endif
