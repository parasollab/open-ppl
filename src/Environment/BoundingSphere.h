#ifndef BOUNDINGSPHERE_H_
#define BOUNDINGSPHERE_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BoundingSphere : public Boundary {
  public:
    BoundingSphere();
    BoundingSphere(const Vector3d& _center, double _radius);
    virtual ~BoundingSphere() = default;

    string Type() const {return "Sphere";}

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
    double m_radius;

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer& _t) {
      _t.member(m_radius);
    }
#endif
};

#ifdef _PARALLEL
namespace stapl {
  template <typename Accessor>
    class proxy<BoundingSphere, Accessor>
    : public Accessor {
      private:
        friend class proxy_core_access;
        typedef BoundingSphere target_t;

      public:
        explicit proxy(Accessor const& acc) : Accessor(acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
        proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
        Point3d GetRandomPoint() const { return Accessor::const_invoke(&target_t::GetRandomPoint);}
    };
}
#endif

#endif
