#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "Vector.h"

#include "Utilities/IOUtils.h"

class Cfg;
class Environment;

using namespace mathtool;

class Boundary {
  public:
    enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};

    Boundary(){}
    virtual ~Boundary(){}

    friend ostream& operator<<(ostream& _os, const Boundary& _b);
    virtual bool operator==(const Boundary& _b) const = 0;

    virtual double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const = 0;
    virtual pair<double, double> GetRange(size_t _i) const = 0;

    virtual Point3d GetRandomPoint() const = 0;
    virtual bool InBoundary(const Vector3d& _p) const = 0;
    virtual double GetClearance(const Vector3d& _p) const = 0;
    virtual int GetSideID(const vector<double>& _p) const = 0;
    virtual Vector3d GetClearancePoint(const Vector3d& _p) const = 0;
    virtual double GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const = 0;

    virtual void ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d) = 0;

    virtual bool IsInterSect(Boundary* _b){ return true;}//not implemented yet
    virtual Boundary* GetIntersect(Boundary* _b){return _b;}//not implemented yet
    virtual bool IsOverlap(Boundary* _b){return true;}//not implemented yet
    virtual Boundary* GetOverlap(Boundary* _b){return _b;}//not implemented yet

    virtual void Read(istream& _is) = 0;
    virtual void Write(ostream& _os) const = 0 ;

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer&) { }
#endif
};

#endif
