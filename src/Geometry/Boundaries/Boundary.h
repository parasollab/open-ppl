#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include "Vector.h"
using namespace mathtool;

#include "Range.h"
#include "Utilities/IOUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

class Cfg;


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for a bounding volume in workspace or c-space.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class Boundary {

  public:

    ///@name Construction
    ///@{

    virtual ~Boundary() = default;

    /// Duplicate this boundary and return a dynamically-allocated copy with the
    /// same type.
    /// @return A dynamically-allocated copy of this with the same type. Memory
    ///         must be released by the user.
    virtual Boundary* Clone() const = 0;

    ///@}
    ///@name Property Accessors
    ///@{

    /// Get the name of the boundary type.
    virtual std::string Type() const noexcept = 0;

    /// Get the longest distance contained within the boundary. Supports any
    /// Minkowski distance.
    /// @param _r1 The term-wise power in the Minkowski difference.
    /// @param _r2 The whole expression power in the Minkowski difference.
    virtual double GetMaxDist(const double _r1 = 2., const double _r2 = .5)
        const = 0;

    /// Get the boundary range for a specific dimension.
    /// @param _i The dimension index.
    /// @return The range of values spanned by this boundary in dimension _i.
    virtual Range<double> GetRange(const size_t _i) const = 0;

    /// Get the boundary's center point.
    virtual const std::vector<double>& GetCenter() const noexcept = 0;

    ///@}
    ///@name Sampling
    ///@{

    /// Get a random point inside the boundary.
    virtual std::vector<double> GetRandomPoint() const = 0;

    ///@}
    ///@name Containment Testing
    ///@{

    /// Test if a specific point lies within the boundary.
    /// @param _p The point to test.
    /// @return True if _p lies inside this boundary.
    virtual bool InBoundary(const Vector3d& _p) const;

    /// Test if a specific n-dimensional point lies within the boundary.
    /// @param _v The point to test.
    /// @return True if _v lies inside this boundary.
    virtual bool InBoundary(const std::vector<double>& _v) const = 0;

    /// Test if a configuration lies within the boundary.
    /// @param _cfg The configuration to test.
    /// @return True if the configuration is considered to lie inside the
    ///         boundary.
    virtual bool InBoundary(const Cfg& _c) const = 0;

    ///@}
    ///@name Clearance Testing
    ///@{

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

    /// Move the boundary to a new center point.
    /// @param _c The new center point for the boundary.
    virtual void SetCenter(const std::vector<double>& _c) noexcept = 0;

    /// Translate the boundary by an offset vector.
    /// @param _v The offset to apply.
    virtual void ApplyOffset(const Vector3d& _v) = 0;

    /// Resize the boundary to fit inside a bounding box plus some margin.
    /// @param _bbx The new base bounding box.
    /// @param _margin The additional margin for _bbx. Negative margins cause
    ///                shrinkage.
    virtual void ResetBoundary(const std::vector<pair<double, double>>& _bbx,
        const double _margin) = 0;

    ///@}
    ///@name I/O
    ///@{

    /// Read in a boundary.
    /// @param _is The input stream to read from.
    /// @param _cbs The counting stream buffer for keeping track of where we are
    ///             in the input stream.
    virtual void Read(std::istream& _is, CountingStreamBuffer& _cbs) = 0;

    /// Write out a boundary.
    /// @param _os The output stream to write to.
    virtual void Write(std::ostream& _os) const = 0 ;

    ///@}
    ///@name CGAL Representations
    ///@{

    typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
    typedef CGAL::Polyhedron_3<CGALKernel> CGALPolyhedron;

    /// Create a CGAL polyhedron representation of this.
    virtual CGALPolyhedron CGAL() const;

    ///@}

  protected:

    ///@name Containment Helpers
    ///@{

    /// Check that a configuration lies entirely within a workspace boundary.
    /// @param _c The configuration of interest.
    /// @return True if all of _c's geometry vertices lie within this boundary.
    bool InWorkspace(const Cfg& _c) const;

    /// Check that a configuration's DOF values lie entirely within a CSpace
    /// boundary.
    /// @param _c The configuration of interest.
    /// @return True if all of _c's DOF values lie within this boundary.
    bool InCSpace(const Cfg& _c) const;

    ///@}

#ifdef _PARALLEL
  public:

    void define_type(stapl::typer&) { }
#endif

};

/*----------------------------------- I/O ------------------------------------*/

ostream& operator<<(ostream& _os, const Boundary& _b);

/// @TODO Move impl from environment to here.
//istream& operator>>(istream& _is, const Boundary& _b);

/*----------------------------------------------------------------------------*/

#endif
