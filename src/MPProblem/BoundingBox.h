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

  bool operator==(const Boundary& bb) const;

  void SetParameter(int par, double p_first, double p_second);
  std::vector<BoundingBox> Partition(int par, double p_point, double epsilon);

  int FindSplitParameter(BoundingBox &o_bounding_box);

  BoundingBox GetCombination(BoundingBox &o_bounding_box);
  double GetClearance(Vector3D point3d) const;
  parameter_type GetType(int par) const;
  Point3d GetRandomPoint();

  void TranslationalScale(double scale_factor);


  void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const;

  //void Parse(std::stringstream &i_bbox);

  bool IfWrap(int par);
  bool IfEnoughRoom(int par, double room);
  bool IfSatisfiesConstraints(Vector3D point3d) const;
  bool IfSatisfiesConstraints(vector<double> cfg) const;
  bool InBoundary(const Cfg& cfg);
 private:
  std::vector<parameter_type> par_type;
  public:
  #ifdef _PARALLEL
  void define_type(stapl::typer &_t)
  {
	  _t.member(par_type);
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
