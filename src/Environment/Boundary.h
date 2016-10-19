#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "Vector.h"
using namespace mathtool;

#include "Utilities/IOUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class Boundary {

  public:

    typedef CGAL::Exact_predicates_exact_constructions_kernel CGALKernel;
    typedef CGAL::Polyhedron_3<CGALKernel> CGALPolyhedron;

    /// Create a CGAL polyhedron.
    virtual CGALPolyhedron CGAL() const {return CGALPolyhedron();}

    Boundary() = default;
    virtual ~Boundary() = default;

    virtual string Type() const = 0;

    const Point3d& GetCenter() const {return m_center;}
    virtual double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const = 0;
    virtual pair<double, double> GetRange(size_t _i) const = 0;

    virtual Point3d GetRandomPoint() const = 0;
    virtual bool InBoundary(const Vector3d& _p) const = 0;
    virtual double GetClearance(const Vector3d& _p) const = 0;
    virtual int GetSideID(const vector<double>& _p) const = 0;
    virtual Vector3d GetClearancePoint(const Vector3d& _p) const = 0;

    virtual double GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const {
      return 0;
    }

    virtual void ApplyOffset(const Vector3d& _v) = 0;
    virtual void ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d) = 0;

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) = 0;
    virtual void Write(ostream& _os) const = 0 ;

    friend ostream& operator<<(ostream& _os, const Boundary& _b);

  protected:

    Point3d m_center;

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer&) { }
#endif
};

#endif
