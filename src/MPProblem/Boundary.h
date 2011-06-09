#ifndef _Boundary_h_
#define _Boundary_h_

/////////////////////////////////////////////////////////////////////////////////////////
//Include mathtool vec
#include "Vector.h"
/////////////////////////////////////////////////////////////////////////////////////////
//Include OBPRM headers
#include "BasicDefns.h"
#include "util.h"

///\todo add MPBaseObject defautl constructor
class Boundary : public MPBaseObject{
 public:
  Boundary();
  virtual ~Boundary();
 private:
};

///\todo add MPBaseObject defautl constructor
class BoundingBox : public Boundary {
 public:
  enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};
  BoundingBox(int i_dofs, int i_pos_dofs);
  BoundingBox(XMLNodeReader& in_Node,MPProblem* in_pproblem);
  BoundingBox(const BoundingBox &from_bbox);
  virtual ~BoundingBox();

  bool operator==(const BoundingBox& bb) const;

  void Clear();
  void SetParameter(int par, double p_first, double p_second);
  std::vector<BoundingBox> Partition(int par, double p_point, double epsilon);

  int FindSplitParameter(BoundingBox &o_bounding_box);

  BoundingBox GetCombination(BoundingBox &o_bounding_box);
  double GetRandomValueInParameter(int par);
  int GetDOFs() const;
  int GetPosDOFs() const;
  const std::pair<double,double> GetRange(int par) const;
  double GetClearance(Vector3D point3d) const;
  parameter_type GetType(int par) const;
  
  void TranslationalScale(double scale_factor);

  void SetRanges(std::vector<double> &ranges);

  void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const;

  //void Parse(std::stringstream &i_bbox);

  bool IfWrap(int par);
  bool IfEnoughRoom(int par, double room);
  bool IfSatisfiesConstraints(Vector3D point3d) const;
  bool IfSatisfiesConstraints(vector<double> cfg) const;

 private:
  std::vector< std::pair<double,double> > bounding_box; // bb size is the dof
  std::vector<parameter_type> par_type;
  int pos_dofs;
  int dofs;
};


#endif /*_Boundary_h_*/
