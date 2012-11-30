#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_

#include "Boundary.h"
//#include "MPUtils.h"

class Environment;

class BoundingBox :  public Boundary {
  public:
    BoundingBox(int _DOFs = 0, int _posDOFs = 0);
    BoundingBox(XMLNodeReader& _node);
    BoundingBox(const BoundingBox& _bbox);
    ~BoundingBox();

    void Clear();
    bool operator==(const Boundary& _bb) const;

    void SetParameter(int _par, double _first, double _second);
    std::vector<BoundingBox> Partition(int _par, double _point, double _epsilon);

    int FindSplitParameter(BoundingBox& _boundingBox);

    BoundingBox GetCombination(BoundingBox& _boundingBox);
    double GetClearance(Vector3D _point3d) const;
    Point3d GetRandomPoint();

    void TranslationalScale(double _scaleFactor);


    void Print(std::ostream& _os, char _rangeSep=':', char _parSep=';') const;

    bool IfWrap(int _par);
    bool IfEnoughRoom(int _par, double _room);
    bool IfSatisfiesConstraints(Vector3D _point3d) const;
    bool IfSatisfiesConstraints(vector<double> _cfg) const;
    bool InBoundary(const Cfg& _cfg, Environment* _env);

  public:
#ifdef _PARALLEL

    void define_type(stapl::typer &_t)
    {
      _t.member(m_jointLimits);
      _t.member(m_boundingBox);
      _t.member(m_posDOFs);
      _t.member(m_DOFs);
      _t.member(m_parType);
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
        typedef target_t::parameter_type  parameter_type;
        explicit proxy(Accessor const& acc) : Accessor(acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
        proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
        Point3d GetRandomPoint() const { return Accessor::const_invoke(&target_t::GetRandomPoint);}
        parameter_type GetType(int _par) const { return Accessor::const_invoke(&target_t::GetType, _par);}
    };
}
#endif

#endif /*BOUNDINGBOX_H_*/
