#ifndef BOUNDINGSPHERE_H_
#define BOUNDINGSPHERE_H_
#include "Boundary.h"
#include "Environment.h"
class Environment;

class MultiBody;
using namespace mathtool;

///\todo add MPBaseObject defautl constructor
class BoundingSphere : public Boundary {
 public:
  enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};
  BoundingSphere(int _dofs, int _posDofs);
  BoundingSphere(XMLNodeReader& _inNode,MPProblem* _inPproblem);
  BoundingSphere(const BoundingSphere &_fromBsphere);
  BoundingSphere();
  virtual ~BoundingSphere();

  bool operator==(const Boundary& _bb) const;
  
  const double GetParameter(int _par) const;
  void SetParameter(int _par, double _pValue);
  void SetParameter(int _par, double _pFirst, double _pSecond);
  std::vector<BoundingSphere> Partition(int _par, double _pPoint, double _epsilon);

  int FindSplitParameter(BoundingSphere &_oBoundingSphere);

  BoundingSphere GetCombination(BoundingSphere &_oBoundingSphere);
  double GetClearance(Vector3D _point3d) const;
  parameter_type GetType(int _par) const;
  Point3d GetRandomPoint();

  void TranslationalScale(double _scaleFactor);


  void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const;

  //void Parse(std::stringstream &i_bbox);

  bool IfWrap(int par);
  bool IfEnoughRoom(int _par, double _room);
  bool IfSatisfiesConstraints(Vector3D _point3d) const;
  bool IfSatisfiesConstraints(vector<double> _cfg) const;
  bool InBoundary(const Cfg& _cfg);
 private:
  std::vector<double> boundingSphere; // bb size is the dof
  std::vector<parameter_type> parType;
  public:
  #ifdef _PARALLEL
  void define_type(stapl::typer &t)
  {
	  t.member(boundingSphere);
	  t.member(parType);
  }
  #endif
};

#endif /*BOUNDINGSPHERE_H_*/
