#ifndef GMS_POLYGON_H_
#define GMS_POLYGON_H_

#include <cstddef>
#include <utility>
#include <vector>

#include "Orientation.h"
#include "Vector.h"
using namespace mathtool;


////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// Geometric structure for polygons, including vertex indexes, normal, and
/// area. The vertex indexes refer to an external vertex list, which is required
/// for accessing the points through this object.
///
/// Since we require triangulated models, this usually represents a triangle.
////////////////////////////////////////////////////////////////////////////////
class GMSPolygon {

  public:

    ///@name Local Types
    ///@{

    typedef typename std::vector<Point3d>      PointList;
    typedef typename std::vector<int>          IndexList;

    typedef typename IndexList::iterator       iterator;
    typedef typename IndexList::const_iterator const_iterator;

    ///@}
    ///@name Construction
    ///@{

    /// Default construction is provided for compatibility with STL
    /// containers only. It should not be called otherwise.
    GMSPolygon() = default;

    /// Construct a polygon (triangle) from three vertex indexes.
    GMSPolygon(int _v1, int _v2, int _v3, const PointList& _pts);

    ///@}
    ///@name Accessors
    ///@}

    iterator begin() {return m_indexes.begin();}
    iterator end() {return m_indexes.end();}

    const_iterator begin() const {return m_indexes.begin();}
    const_iterator end() const {return m_indexes.end();}

    int& operator[](const size_t _i) {return m_indexes[_i];}
    const int& operator[](const size_t _i) const {return m_indexes[_i];}

    const size_t GetNumVertices() const {return m_indexes.size();}

    /// Get the actual point referenced by the _i'th index in the vertex list.
    const Point3d& GetPoint(const size_t _i) const;

    Vector3d& GetNormal() {return m_normal;}
    const Vector3d& GetNormal() const {return m_normal;}

    const double GetArea() const {return m_area;}

    ///@}
    ///@name Modifiers
    ///@{

    void Reverse(); ///< Reverse the facing of this polygon.
    void ComputeNormal(); ///< Compute the normal and area for this polygon.

    ///@}
    ///@name Equality
    ///@{

    const bool operator==(const GMSPolygon& _p) const;
    const bool operator!=(const GMSPolygon& _p) const;

    ///@}
    ///@name Queries
    ///@{

    const bool IsTriangle() const; ///< Test for three unique vertex indexes.

    /// Find the centroid of this polygon.
    const Point3d FindCenter() const;

    /// Test whether a point lies above or below the polygon plane. A
    /// point is considered above the plane when it is on the side where
    /// the normal faces outward.
    /// @param[in] _p The point to check.
    const bool PointIsAbove(const Point3d& _p) const;

    /// Find a common vertex between two polygons.
    /// @param[in] _p The other polygon under consideration.
    /// @return A common vertex index, or -1 if none exists.
    const int CommonVertex(const GMSPolygon& _p) const;

    /// Find the common edge between two polygons.
    /// @param[in] _p The other polygon under consideration.
    /// @return A pair of vertex indexes if a common edge exists. If not, at
    ///         least one of the indexes will be -1.
    const std::pair<int, int> CommonEdge(const GMSPolygon& _p) const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    IndexList m_indexes;  ///< The vertex indexes in this polygon.
    Vector3d m_normal;    ///< The normal vector of this polygon.
    double m_area{0};     ///< Area of this polygon.

    /// The structure holding the points referenced by m_indexes.
    const PointList* m_pointList{nullptr};

    ///@}
};

#endif
