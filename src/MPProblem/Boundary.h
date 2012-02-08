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
  virtual ~Boundary();
  virtual double GetClearance(Vector3D _point3d) const = 0;
  virtual bool IfSatisfiesConstraints(Vector3D _point3d) const =0;
  virtual bool InBoundary(const Cfg& _cfg) = 0; 
  virtual Point3d GetRandomPoint() = 0; 
  double GetRandomValueInParameter(int _par);
  virtual bool IsInterSect(Boundary* _b){ return true;}//not implemented yet
  virtual Boundary* GetIntersect(Boundary* _b){return _b;}//not implemented yet
  virtual bool IsOverlap(Boundary* _b){return true;}//not implemented yet
  virtual Boundary* GetOverlap(Boundary* _b){return _b;}//not implemented yet
  virtual void Print(std::ostream& _os, char range_sep=':', char par_sep=';') const=0 ;
 protected:
 vector< std::pair<double,double> > m_jointLimits; 
};

#endif /*_Boundary_h_*/
