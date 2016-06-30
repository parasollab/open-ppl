#ifndef BOUNDING_BOX_2D_H_
#define BOUNDING_BOX_2D_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BoundingBox2D :  public Boundary {
  public:
    BoundingBox2D();
    BoundingBox2D(pair<double, double> _x,
        pair<double, double> _y);
    ~BoundingBox2D() {}

    string Type() const {return "Box2D";}

    double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const;
    pair<double, double> GetRange(size_t _i) const;

    Point3d GetRandomPoint() const;
    bool InBoundary(const Vector3d& _p) const;
    double GetClearance(const Vector3d& _p) const;
    int GetSideID(const vector<double>& _p) const;
    Vector3d GetClearancePoint(const Vector3d& _p) const;
    double GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const;

    void ApplyOffset(const Vector3d& _v);
    void ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d);

    void Read(istream& _is, CountingStreamBuffer& _cbs);
    void Write(ostream& _os) const;

    virtual CGALPolyhedron CGAL() const override;

  private:
    pair<double, double> m_bbx[2];
};

#endif
