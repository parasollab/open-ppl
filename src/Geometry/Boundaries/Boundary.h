#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "Vector.h"
using namespace mathtool;

#include "Utilities/IOUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

class Cfg;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Geometry
/// A general boundary object.
////////////////////////////////////////////////////////////////////////////////
class Boundary {

  public:

    ///@name Construction
    ///@{

    Boundary() = default;

    virtual ~Boundary() = default;

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the name of the boundary type.
    virtual string Type() const noexcept = 0;

    /// Get the center point of the boundary.
    const Point3d& GetCenter() const noexcept;

    /// Get the longest distance contained within the boundary. Supports any
    /// Minkowski distance.
    /// @param _r1 The term-wise power in the Minkowski difference.
    /// @param _r2 The whole expression power in the Minkowski difference.
    virtual double GetMaxDist(double _r1 = 2., double _r2 = .5) const = 0;

    /// Get the boundary range for a specific dimension.
    /// @param _i The dimension index.
    /// @return The range of values spanned by this boundary in dimension _i.
    virtual pair<double, double> GetRange(size_t _i) const = 0;

    ///@}
    ///@name Sampling
    ///@{

    /// Get a random point inside the boundary.
    virtual Point3d GetRandomPoint() const = 0;

    ///@}
    ///@name Containment Testing
    ///@{

    /// Test if a specific point lies within the boundary.
    /// @param _p The point to test.
    /// @return True if _p lies inside this boundary.
    virtual bool InBoundary(const Vector3d& _p) const = 0;

    /// Test if every point of an object configuration lies within the boundary.
    /// @param _cfg The configuration to test.
    /// @return True if all points of the object represented by _c lies inside
    ///         this boundary.
    virtual bool InBoundary(const Cfg& _c) const;

    /// Test if each DOF value of a given configuration lies within the
    /// boundary. If the configuration has more DOF than the boundary has
    /// dimensions, then the unbounded dimensions will be assumed infinite.
    /// @warning This does NOT imply that the object is inside the boundary - it
    ///          is a test for the c-space reference point only.
    /// @param[in] _c The configuration to test.
    /// @return True if _c's DOF values lie within this boundary.
    virtual bool InCSpace(const Cfg& _c) const;

    ///@}
    ///@name Clearance Testing
    ///@{

    /// Get the id of the
    virtual int GetSideID(const vector<double>& _p) const = 0;

    /// Get the distance from a test point to the nearest point on the boundary.
    /// @param _p The test point.
    /// @return The distance from _p to the clearance point of _p.
    virtual double GetClearance(const Vector3d& _p) const = 0;

    /// Get the nearest point on the boundary to a test point.
    /// @param _p The test point.
    /// @return The point on this boundary nearest to _p.
    virtual Vector3d GetClearancePoint(const Vector3d& _p) const = 0;

    ///@}
    ///@name Modifiers
    ///@{

    /// Translate the boundary by an offset vector.
    /// @param _v The offset to apply.
    virtual void ApplyOffset(const Vector3d& _v) = 0;

    /// Resize the boundary to fit inside a bounding box plus some margin.
    /// @param _bbx The new base bounding box.
    /// @param _margin The additional margin for _bbx. Negative margins cause
    ///                shrinkage.
    virtual void ResetBoundary(const vector<pair<double, double>>& _bbx,
        double _margin) = 0;

    ///@}
    ///@name I/O
    ///@{

    /// Read in a boundary.
    /// @param _is The input stream to read from.
    /// @param _cbs The counting stream buffer for keeping track of where we are
    ///             in the input stream.
    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) = 0;

    /// Write out a boundary.
    /// @param _os The output stream to write to.
    virtual void Write(ostream& _os) const = 0 ;

    ///@}
    ///@name CGAL Representations
    ///@{

    typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
    typedef CGAL::Polyhedron_3<CGALKernel> CGALPolyhedron;

    /// Create a CGAL polyhedron representation of this.
    virtual CGALPolyhedron CGAL() const;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Point3d m_center; ///< The center-point of the boundary.

    ///@}

#ifdef _PARALLEL
  public:

    void define_type(stapl::typer&) { }
#endif

};

/*--------------------------------- Display ----------------------------------*/

ostream& operator<<(ostream& _os, const Boundary& _b);

/*----------------------------------------------------------------------------*/

#endif
