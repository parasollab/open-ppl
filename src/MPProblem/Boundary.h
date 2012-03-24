#ifndef BOUNDARY_H_
#define BOUNDARY_H_

/////////////////////////////////////////////////////////////////////////////////////////
//Include mathtool vec
#include "Vector.h"
/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "MPUtils.h"
#include "Point.h"
using namespace mathtool;

///\todo add MPBaseObject defautl constructor
class Boundary : public MPBaseObject{
 public:
  Boundary();
  Boundary(XMLNodeReader& in_Node,MPProblem* in_pproblem);
  //virtual ~Boundary();
  virtual double GetClearance(Vector3D _point3d) const = 0;
  virtual bool IfSatisfiesConstraints(Vector3D _point3d) const =0;
  virtual bool InBoundary(const Cfg& _cfg, Environment* _env = NULL) = 0; 
  virtual Point3d GetRandomPoint() = 0; 
  double GetRandomValueInParameter(int _par);
  virtual bool IsInterSect(Boundary* _b){ return true;}//not implemented yet
  virtual Boundary* GetIntersect(Boundary* _b){return _b;}//not implemented yet
  virtual bool IsOverlap(Boundary* _b){return true;}//not implemented yet
  virtual Boundary* GetOverlap(Boundary* _b){return _b;}//not implemented yet
  virtual void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const=0 ;
 protected:
 vector< std::pair<double,double> > m_jointLimits; 
 
 #ifdef _PARALLEL
 void define_type(stapl::typer &_t){
	_t.member(m_jointLimits);
 }
 #endif
};

#ifdef _PARALLEL
namespace stapl {
template <typename Accessor>
class proxy<Boundary, Accessor> 
: public Accessor {
private:
  friend class proxy_core_access;
  typedef Boundary target_t;
  
public:
  explicit proxy(Accessor const& acc) : Accessor(acc) { }
  //operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
  double GetRandomValueInParameter(int _par) { return Accessor::invoke(&target_t::GetRandomValueInParameter, _par);}
};
}
#endif

#endif /*_Boundary_h_*/
