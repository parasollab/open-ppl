#ifndef BOUNDING_BOX_H_
#define BOUNDING_BOX_H_

#include "Boundary.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Environment
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class BoundingBox :  public Boundary {
  public:
    BoundingBox();
    BoundingBox(pair<double, double> _x,
        pair<double, double> _y,
        pair<double, double> _z);
    ~BoundingBox() {}

    string Type() const {return "Box";}

    const pair<double, double>* const GetBox() const {return m_bbx;}

    double GetMaxDist(double _r1 = 2.0, double _r2 = 0.5) const;
    pair<double, double>& GetRange(size_t _i);
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
    pair<double, double> m_bbx[3];

#ifdef _PARALLEL
  public:
    void define_type(stapl::typer &_t)
    {
      _t.member(m_bbx);
    }
#endif
};

#ifdef _PARALLEL
namespace stapl {
  template <typename Accessor>
    class proxy<BoundingBox, Accessor>
    : public Accessor {
      private:
        friend class proxy_core_access;
        typedef BoundingBox target_t;

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
