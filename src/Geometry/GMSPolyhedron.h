#ifndef PMPL_GMS_POLYHEDRON_H_
#define PMPL_GMS_POLYHEDRON_H_

#include "GMSPolygon.h"

#include "Transformation.h"
#include "Vector.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include <iostream>
#include <string>
#include <utility>
#include <vector>

class IModel;
class PQP_Model;
class RAPID_model;

using namespace mathtool;
namespace glutils {
  class triangulated_model;
}


////////////////////////////////////////////////////////////////////////////////
/// Geometric structure for polyhedra including vertices, faces, and surface
/// area.
///
/// @TODO Replace this object with CGAL::Polyhedron_3 or at least extend from
///       it. We really do not need three different classes to represent
///       polyhedrons, and CGAL's representation is the most mature.
///
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron final {

  public:

    ///@name Local Types
    ///@{

    typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
    typedef CGALKernel::Point_3                               CGALPoint;
    typedef CGAL::Polyhedron_3<CGALKernel>                    CGALPolyhedron;

    ////////////////////////////////////////////////////////////////////////////
    /// Center of mass adjustment approaches
    ///
    /// This enum lists the method to adjust all vertices of a model. 'COM' will
    /// subtract the center of mass (com) from all vertices. 'Surface' will
    /// subtract only x and z components of the com from all vertices. 'None'
    /// will not perform any adjustment.
    ///
    /// @todo This has been a major source of bugs for a long time and should be
    ///       removed entirely. All polyhedrons should be bounding-box centered
    ///       without exception.
    ////////////////////////////////////////////////////////////////////////////
    enum class COMAdjust {COM, Surface, None};

    ///@}
    ///@name Construction
    ///@{

    GMSPolyhedron();

    GMSPolyhedron(const GMSPolyhedron& _p);

    GMSPolyhedron(GMSPolyhedron&& _p);

    GMSPolyhedron(glutils::triangulated_model&& _t);

    ~GMSPolyhedron();

    ///@}
    ///@name Assignment
    ///@{

    GMSPolyhedron& operator=(const GMSPolyhedron& _p);
    GMSPolyhedron& operator=(GMSPolyhedron&& _p);

    ///@}
    ///@name Transformation
    ///@{

    /// Apply a transformation to the polyhedron.
    GMSPolyhedron& operator*=(const Transformation& _t);

    /// Invert the polyhedron so that normals face the opposite direction.
    void Invert();

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const GMSPolyhedron& _p) const;
    bool operator!=(const GMSPolyhedron& _p) const;

    ///@}
    ///@name IO Functions
    ///@{

    /// Read in a geometry file in either BYU or OBJ format.
    /// @param _fileName The name of the file to read.
    /// @param _comAdjust The type of COM adjustment to use.
    /// @return The parsed model's center of mass.
    Vector3d Read(const std::string& _fileName, COMAdjust _comAdjust);

    /// Load vertices and triangles from the IModel, which loads all types
    /// of models.
    /// @param _imodel An IModel input.
    /// @param_comAdjust The COM adjustment type to use.
    /// @return The parsed model's center of mass.
    Vector3d LoadFromIModel(IModel* _imodel, COMAdjust _comAdjust);

    /// Output the model to a BYU-format file.
    /// @param _os The output stream to use.
    void WriteBYU(std::ostream& _os) const;

    /// Output the model to a Obj-format file.
    /// @param _os The output stream to use.
    void WriteObj(std::ostream& _os) const;

    ///@}
    ///@name Accessors
    ///@{

    std::vector<Vector3d>& GetVertexList() noexcept;
    std::vector<GMSPolygon>& GetPolygonList() noexcept;

    const std::vector<Vector3d>& GetVertexList() const noexcept;
    const std::vector<GMSPolygon>& GetPolygonList() const noexcept;

    /// Get the boundary edges for this polyhedron. The edges will be computed
    /// if they haven't been already.
    std::vector<std::pair<int,int>>& GetBoundaryLines();

    ///@}
    ///@name Geometry Functions
    ///@{

    /// Get a random point on the surface of the polyhedron.
    Point3d GetRandPtOnSurface() const;

    /// Get the centroid of the polyhedron (average of the vertices).
    const Vector3d& GetCentroid() const;

    /// Mark all cached objects as requiring an update.
    void MarkDirty();

    /// Compute a GMSPolyhedron representation of an axis-aligned bounding box
    /// for this.
    ///
    /// Vertex diagram:
    ///
    ///     2-----6    +Y
    ///    /|    /|     |
    ///   3-----7 |     |---+X
    ///   | 0---|-4    /
    ///   |/    |/   +Z
    ///   1-----5
    ///
    GMSPolyhedron ComputeBoundingPolyhedron() const;

    /// Compute the polyhedron's convex hull vertices and facets
    /// @param _convexHull convexhull container.
    ///
    GMSPolyhedron ComputeConvexHull() const;

    /// Get a CGAL polyhedron representation of this object.
    CGALPolyhedron CGAL() const;

    /// Make sure the CGAL points match the vertex list. This is only needed for
    /// polys that are created manually rather than from file. The points will
    /// not be exact as they are copied from doubles.
    void UpdateCGALPoints();

    /// Scale a polyhedron by a factor and keep it centered at its original
    /// centroid.
    /// @param _scalingFactor The scaling factor
    void Scale(double _scalingFactor);

    /// Get the total surface area of the polyhedron.
    double GetSurfaceArea() const noexcept;

    /// Get the maximum radius relative to the polyhedron's center.
    double GetMaxRadius() const noexcept;

    /// Get the minimum radius relative to the polyhedron's center.
    double GetMinRadius() const noexcept;

    ///@}
    ///@name Collision Detection Models
    ///@{

    /// Build rapid and PQP models of this polyhedron.
    void BuildCDModels();

    /// Get the rapid CD model.
    RAPID_model* GetRapidModel() const noexcept;

    /// Get the pqp CD model.
    PQP_Model* GetPQPModel() const noexcept;

    ///@}

  private:

    ///@name Initialization Helpers
    ///@{

    /// Construct the list of external edges.
    void BuildBoundaryLines();

    /// Compute the centroid.
    void ComputeCentroid() const;

    /// Sort the polygons by decreasing order.
    void OrderFacets();

    /// Compute the surface area.
    void ComputeSurfaceArea();

    /// Compute the minimum and maximum radius relative to model coordinates.
    /// @WARNING The computed values are relative to the model's local frame.
    void ComputeRadii();

    ///@}
    ///@name Internal State
    ///@{

    std::vector<Vector3d> m_vertexList;    ///< Vertices in this polyhedron.
    std::vector<CGALPoint> m_cgalPoints;   ///< Exact representation of vertices.
    std::vector<GMSPolygon> m_polygonList; ///< Boundary faces of this polyhedron.

    double m_area{0};      ///< The total area of polygons in this polyhedron.
    double m_maxRadius{0}; ///< The maximum distance from a vertex to COM.
    double m_minRadius{0}; ///< The minimum distance from a vertex to COM.

    Vector3d m_centroid;          ///< The polyhedron centroid (avg of vertices).
    mutable bool m_centroidCached{false}; ///< Is the centroid cached?

    std::vector<std::pair<int,int>> m_boundaryLines; ///< Surface edges.

    std::unique_ptr<RAPID_model> m_rapidModel;  ///< RAPID model
    std::unique_ptr<PQP_Model> m_pqpModel;      ///< PQP model

    ///@}

};

/*----------------------------- Transformation -------------------------------*/

GMSPolyhedron
operator*(const Transformation& _t, const GMSPolyhedron& _poly);

/*----------------------------------------------------------------------------*/

#endif
