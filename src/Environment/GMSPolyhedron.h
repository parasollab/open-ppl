#ifndef GMSPOLYHEDRON_H_
#define GMSPOLYHEDRON_H_

#include <iostream>
#include <string>
#include <vector>
using namespace std;

#include "Vector.h"
using namespace mathtool;

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>

class IModel;

#include "GMSPolygon.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief Geometric structure for polyhedra.
///
/// Contains vertices, faces, normals, and surface area of this polyhedra.
////////////////////////////////////////////////////////////////////////////////
class GMSPolyhedron {

  public:

    ///@name Local Types
    ///@{

    typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
    typedef CGALKernel::Point_3                               CGALPoint;
    typedef CGAL::Polyhedron_3<CGALKernel>                    CGALPolyhedron;

    ////////////////////////////////////////////////////////////////////////////
    /// @ingroup Environment
    /// @brief Center of mass adjustment approaches
    ///
    /// This enum lists the method to adjust all vertices of a model. 'COM' will
    /// subtract the center of mass (com) from all vertices. 'Surface' will
    /// subtract only x and z components of the com from all vertices. 'None'
    /// will not perform any adjustment.
    ////////////////////////////////////////////////////////////////////////////
    enum class COMAdjust {COM, Surface, None};

    ///@}
    ///@name Equality
    ///@{

    bool operator==(const GMSPolyhedron& _p) const;
    bool operator!=(const GMSPolyhedron& _p) const;

    ///@}
    ///@name IO Functions
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read in a geometry file in either BYU or OBJ format.
    /// @param[in] _fileName The name of the file to read.
    /// @param[in] _comAdjust The type of COM adjustment to use.
    /// @return The parsed model's center of mass.
    Vector3d Read(string _fileName, COMAdjust _comAdjust);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Load vertices and triangles from the IModel, which loads all types
    ///        of models
    /// @param[in] _imodel An IModel input.
    /// @param[in] _comAdjust The COM adjustment type to use.
    /// @return The parsed model's center of mass.
    Vector3d LoadFromIModel(IModel* _imodel, COMAdjust _comAdjust);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Output the model to a BYU-format file.
    /// @param[in] _os The output stream to use.
    void WriteBYU(ostream& _os) const;

    ///@}
    ///@name Accessors
    ///@{

    vector<Vector3d>& GetVertexList() {return m_vertexList;}
    vector<GMSPolygon>& GetPolygonList() {return m_polygonList;}

    const vector<Vector3d>& GetVertexList() const {return m_vertexList;}
    const vector<GMSPolygon>& GetPolygonList() const {return m_polygonList;}

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the boundary edges for this polyhedron. The edges will be
    ///        computed if they haven't been already.
    vector<pair<int,int>>& GetBoundaryLines() {
       BuildBoundary();
       return m_boundaryLines;
    };

    ///@}
    ///@name Geometry Functions
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get a random point on the surface of the polyhedron.
    Point3d GetRandPtOnSurface() const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Check if a 2d point lies on the surface of the polyhedron.
    /// @param[in] _p The point of interest.
    bool IsOnSurface(const Point2d& _p) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get the height (y-coord) of the polyhedron at a designated point
    ///        on the xz-plane..
    /// @param[in] _p The xz point.
    /// @param[in/out] _valid Does the polyhedron have a point at _p?
    /// @return The height of the polyhedron at _p, or -19999 if invalid.
    double HeightAtPt(const Point2d& _p, bool& _valid) const;

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Compute the closest point on a boundary edge of the polyhedron to
    ///        a reference point _p.
    /// @param[in] _p The reference point of interest.
    /// @param[in/out] _closest The closest point on the bounding edges.
    /// @return The distance from _p to _closest.
    /// @warning This function doesn't compute the clearance to the polyhedron:
    ///          instead, it computes the clearance to the closest point on a
    ///          boundary edge.
    double GetClearance(const Point3d& _p, Point3d& _closest);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Push a point to the (surface) medial axis.
    /// @param[in] _p The point to push.
    /// @return The pushed point's clearance from the nearest obstacle.
    /// @warning This function is mis-named: it is actually a push to to the
    ///          'surface' medial axis on the xz plane. GB people, please correct
    ///          this as soon as convenience allows.
    double PushToMedialAxis(Point3d& _p);

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Get a CGAL polyhedron representation of this object.
    CGALPolyhedron CGAL() const;

    ///@}
    ///@name Internal State
    ///@{

    vector<Vector3d> m_vertexList;    ///< Vertices in this polyhedron.
    vector<CGALPoint> m_cgalPoints;   ///< Exact CGAL representation of vertices.
    vector<GMSPolygon> m_polygonList; ///< Boundary faces of this polyhedron.

    double m_area{0};      ///< The total area of polygons in this polyhedron.
    double m_maxRadius{0}; ///< The maximum distance from a vertex to COM.
    double m_minRadius{0}; ///< The minimum distance from a vertex to COM.

    vector<pair<int,int>> m_boundaryLines; ///< Surface edges.

    bool m_boundaryBuilt{false};    ///< Is the boundary initialized?
    bool m_force2DBoundary{false};  ///< Require a 2d boundary.

    ///@}

  private:

    ///@name Initialization Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Construct the list of external edges.
    void BuildBoundary();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief As BuildBoundary, but saving only edges near to the XZ plane.
    void BuildBoundary2D();

    ///@}
};

#endif
