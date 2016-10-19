#ifndef WORKSPACE_REGION_H_
#define WORKSPACE_REGION_H_

#include <memory>
#include <vector>

#include "Environment/GMSPolyhedron.h"

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

    ////////////////////////////////////////////////////////////////////////////
    /// @warning Default construction makes no sense for this object, but the
    ///          stapl graph requires it.
    WorkspaceRegion() : m_decomposition(nullptr) {}

    WorkspaceRegion(WorkspaceDecomposition* const _wd) : m_decomposition(_wd) {}

    ///@}
    ///@name Modifiers
    ///@{

    void AddPoint(const size_t _i);
    void AddFacet(Facet&& _f);

    /// Add a boundary and assume ownership of it.
    void AddBoundary(Boundary* _b);

    ///@}
    ///@name Accessors
    ///@{

    const size_t GetNumPoints() const {return m_points.size();}
    const size_t GetNumFacets() const {return m_facets.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the _i'th point of this region.
    const Point3d& GetPoint(const size_t _i) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get a list of all points in this region.
    const vector<Point3d> GetPoints() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the set of facets that border this region.
    const vector<Facet>& GetFacets() const {return m_facets;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the boundary for this region.
    const Boundary* GetBoundary() const {return m_boundary.get();}

    ///@}
    ///@name Queries
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if a given point is part of this region's boundary.
    const bool HasPoint(const Point3d& _p) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find the center of this region.
    const Point3d FindCenter() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find shared points with another workspace region.
    const vector<Point3d> FindSharedPoints(const WorkspaceRegion& _wr) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Find shared facets with another workspace region.
    const vector<const Facet*> FindSharedFacets(const WorkspaceRegion& _wr) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    WorkspaceDecomposition* const m_decomposition; ///< The owning decomposition.

    vector<size_t> m_points; ///< The indexes of the points bounding this region.
    vector<Facet> m_facets;  ///< The bounding facets of this region.

    shared_ptr<Boundary> m_boundary;  ///< The boundary object for this region.

    ///@}
};

#endif
