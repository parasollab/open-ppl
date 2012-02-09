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
  virtual ~BoundingBox();

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
  bool InBoundary(const Cfg& cfg);
 private:
  std::vector< std::pair<double,double> > bounding_box; // bb size is the dof
  std::vector<parameter_type> par_type;
  int pos_dofs;
  int dofs;
  public:
  #ifdef _PARALLEL
  void define_type(stapl::typer &t)
  {
	  t.member(bounding_box);
	  t.member(par_type);
	  t.member(pos_dofs);
	  t.member(dofs);
  }
  #endif
};

#endif /*BOUNDINGBOX_H_*/
