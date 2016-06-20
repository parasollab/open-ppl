#ifndef BOUNDING_SPHERE_2D_H_
#define BOUNDING_SPHERE_2D_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BoundingSphere2D : public Boundary {
  public:
    BoundingSphere2D();
    BoundingSphere2D(const Vector2d& _center, double _radius);
    ~BoundingSphere2D() {}

    string Type() const {return "Sphere2D";}

    const Vector3d& GetCenter() const {return m_center;}
    const double GetRadius() const {return m_radius;}

    bool operator==(const Boundary& _b) const;

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

  private:
    Vector3d m_center;
    double m_radius;
};

#endif
