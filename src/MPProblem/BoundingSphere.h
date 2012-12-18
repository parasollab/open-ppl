#ifndef BOUNDINGSPHERE_H_
#define BOUNDINGSPHERE_H_
#include "Boundary.h"
#include "Environment.h"
class Environment;

using namespace mathtool;
class MultiBody;
class BoundingBox;
///\todo add MPBaseObject defautl constructor
class BoundingSphere : public Boundary {
 public:
  BoundingSphere(int _dofs, int _posDofs);
  BoundingSphere(XMLNodeReader& _node);
  BoundingSphere(const BoundingSphere& _fromBsphere);
  BoundingSphere();
  virtual ~BoundingSphere();

  bool operator==(const Boundary& _bb) const;
  
  const double GetParameter(int _par) const;
  void SetParameter(int _par, double _pValue);
  void SetParameter(int _par, double _pFirst, double _pSecond);
  std::vector<BoundingSphere> Partition(int _par, double _pPoint, double _epsilon);

  int FindSplitParameter(BoundingSphere& _boundingSphere);

  BoundingSphere GetCombination(BoundingSphere& _boundingSphere);
  double GetClearance(Vector3D _point3d) const;
  Point3d GetRandomPoint();

  void TranslationalScale(double _scaleFactor);


  void Print(std::ostream& _os, char _rangeSep=':', char _parSep=';') const;

  //void Parse(std::stringstream &i_bbox);

  bool IfWrap(int par);
  bool IfEnoughRoom(int _par, double _room);
  bool IfSatisfiesConstraints(Vector3D _point3d) const;
  bool IfSatisfiesConstraints(vector<double> _cfg) const;
  bool InBoundary(const Cfg& _cfg, Environment* _env);

  private:
    std::vector< double > m_boundingSphere;
public:
#ifdef _PARALLEL
  
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
