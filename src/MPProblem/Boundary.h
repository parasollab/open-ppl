#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "Vector.h"
using namespace mathtool;

#include "Utilities/IOUtils.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class Boundary {
  public:
    Boundary() {}
    virtual ~Boundary() {}

    virtual double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const = 0;
    virtual pair<double, double> GetRange(size_t _i) const = 0;

    virtual Point3d GetRandomPoint() const = 0;
    virtual bool InBoundary(const Vector3d& _p) const = 0;
    virtual double GetClearance(const Vector3d& _p) const = 0;
    virtual int GetSideID(const vector<double>& _p) const = 0;
    virtual Vector3d GetClearancePoint(const Vector3d& _p) const = 0;
    virtual double GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const = 0;

    virtual void ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d) = 0;

    virtual void Read(istream& _is, CountingStreamBuffer& _cbs) = 0;
    virtual void Write(ostream& _os) const = 0 ;

    friend ostream& operator<<(ostream& _os, const Boundary& _b);

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer&) { }
#endif
};

#endif
