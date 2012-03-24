#ifndef BOUNDINGBOX_H_
#define BOUNDINGBOX_H_
#include "Boundary.h"
#include "Environment.h"
class Environment;

class MultiBody;
using namespace mathtool;

///\todo add MPBaseObject defautl constructor
class BoundingBox : public Boundary {
 public:
  enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};
  BoundingBox(int i_dofs, int i_pos_dofs);
  BoundingBox(XMLNodeReader& in_Node,MPProblem* in_pproblem);
  BoundingBox(const BoundingBox &from_bbox);
  BoundingBox();
  /*virtual ~BoundingBox();*/
  
  void Clear();

  bool operator==(const BoundingBox& bb) const;

  void SetParameter(int par, double p_first, double p_second);
  std::vector<BoundingBox> Partition(int par, double p_point, double epsilon);

  int FindSplitParameter(BoundingBox &o_bounding_box);

  BoundingBox GetCombination(BoundingBox &o_bounding_box);
  int GetDOFs() const;
  int GetPosDOFs() const;
  const std::pair<double,double> GetRange(int par) const;
  double GetClearance(Vector3D point3d) const;
  parameter_type GetType(int par) const;
  Point3d GetRandomPoint();

  void TranslationalScale(double scale_factor);

  void SetRanges(std::vector<double> &ranges);

  void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const;

  //void Parse(std::stringstream &i_bbox);

  bool IfWrap(int par);
  bool IfEnoughRoom(int par, double room);
  bool IfSatisfiesConstraints(Vector3D point3d) const;
  bool IfSatisfiesConstraints(vector<double> cfg) const;
  bool InBoundary(const Cfg& cfg, Environment* _env );
 private:
  std::vector< std::pair<double,double> > bounding_box; // bb size is the dof
  std::vector<parameter_type> par_type;
  int pos_dofs;
  int dofs;
  public:
  #ifdef _PARALLEL
  void define_type(stapl::typer &_t)
  {
	  _t.member(bounding_box);
	  _t.member(par_type);
	  _t.member(pos_dofs);
	  _t.member(dofs);
	 _t.member(m_jointLimits);
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
  int GetDOFs() const { return Accessor::const_invoke(&target_t::GetDOFs);}
  int GetPosDOFs() const { return Accessor::const_invoke(&target_t::GetPosDOFs);}
  Point3d GetRandomPoint() const { return Accessor::const_invoke(&target_t::GetRandomPoint);}
  parameter_type GetType(int _par) const { return Accessor::const_invoke(&target_t::GetType, _par);}
 // bool InBoundary(const Cfg& _cfg) { return Accessor::invoke(&target_t::InBoundary, _cfg);}
};
}
#endif

#endif /*BOUNDINGBOX_H_*/
