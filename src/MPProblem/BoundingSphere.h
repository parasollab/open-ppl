#ifndef BOUNDINGSPHERE_H_
#define BOUNDINGSPHERE_H_

#include "Boundary.h"

class BoundingSphere : public Boundary {
  public:
    BoundingSphere();
    BoundingSphere(const Vector3d& _center, double _radius);
    BoundingSphere(const BoundingSphere& _bs);
    ~BoundingSphere() {}

    bool operator==(const Boundary& _b) const;

    double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const;
    pair<double, double> GetRange(size_t _i) const;

    Point3d GetRandomPoint() const;
    bool InBoundary(const Vector3d& _p) const;
    double GetClearance(const Vector3d& _p) const;
    int GetSideID(const vector<double>& _p) const;
    Vector3d GetClearancePoint(const Vector3d& _p) const;
    double GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const;

    void ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d);

    void Read(istream& _is);
    void Write(ostream& _os) const;

  private:
    Vector3d m_center;
    double m_radius;

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer &_t)
    {
      _t.member(m_jointLimits);
      _t.member(m_boundingSphere);
      _t.member(m_posDOFs);
      _t.member(m_DOFs);
      _t.member(m_parType);
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
        //enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};
        typedef target_t::parameter_type  parameter_type;
        explicit proxy(Accessor const& acc) : Accessor(acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
        proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
        Point3d GetRandomPoint() const { return Accessor::const_invoke(&target_t::GetRandomPoint);}
        parameter_type GetType(int _par) const { return Accessor::const_invoke(&target_t::GetType, _par);}
    };

  template<>
    struct rmi_call_traits<Boundary> {
      typedef callable_types_list<BoundingBox, BoundingSphere> polymorphic_callable;
    };
}
#endif

#endif /*BOUNDINGSPHERE_H_*/
