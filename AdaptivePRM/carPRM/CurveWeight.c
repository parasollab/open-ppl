/////////////////////////////////////////////////////////////////////
/********************************************************************
 *@file CurveWeight.c
 *@author Shawna Thomas
 *
 * Implementation of CurveWeight class for carlike robots
 *
 *@date  6/25/02
 *******************************************************************/

#include "CurveWeight.h"
#include "OBPRM.h"


///////////////////////////////////////////////////////
//
// CurveWeightFactory
//
///////////////////////////////////////////////////////

bool CurveWeightFactory::Create( IWeight ** ppIWeight /*in/out*/ ) const {
  //check input
  if ( ppIWeight == NULL ) return false;
  *ppIWeight = new CurveWeight();
  if ( *ppIWeight == NULL ) return false; //not enough memory
  return true;
}


///////////////////////////////////////////////////////
//
// CurveWeight
//
///////////////////////////////////////////////////////

CurveWeight::CurveWeight() : PriorityWeight() {}

CurveWeight::CurveWeight(int _lp, double _weight, const Cfg& _midpoint, const Vector3D& _center, 
	      double _radius, double _rotateAngle, int _direction){
  lp = _lp;
  weight = _weight;
  level = 0;
  midpoint = _midpoint;
  center = _center;
  radius = _radius;
  rotateAngle = _rotateAngle;
  direction = _direction;
}

CurveWeight::CurveWeight(int _lp, double _weight, int _level, const Cfg& _midpoint, const Vector3D& _center,
	      double _radius, double _rotateAngle, int _direction){
  lp = _lp;
  weight = _weight;
  level = _level;
  midpoint = _midpoint;
  center = _center;
  radius = _radius;
  rotateAngle = _rotateAngle;
  direction = _direction;
}

IWeight*
CurveWeight::clone() const {
  CurveWeight* pTmp = new CurveWeight();

  pTmp->Weight() = weight;
  pTmp->LP() = lp;
  pTmp->Level() = level;
  pTmp->Midpoint() = midpoint;
  pTmp->Center() = center;
  pTmp->Radius() = radius;
  pTmp->RotateAngle() = rotateAngle;
  pTmp->Direction() = direction;

  return pTmp;
}

void
CurveWeight::Input(istream& in) {
  in >> lp >> weight >> level;
  midpoint.Read(in);
  in >> center >> radius >> rotateAngle >> direction;
}

void
CurveWeight::Output(ostream& out) const {
  out << lp << " " << weight << " " << level << " ";
  midpoint.Write(out);
  out << " " << center << " " << radius << " " << rotateAngle << " " << direction << " ";
}

bool 
CurveWeight::operator== (const CurveWeight& tmp) const {
  return (lp == tmp.lp && weight == tmp.weight && level == tmp.level && 
	  midpoint == tmp.midpoint && center == tmp.center && radius == tmp.radius && 
	  rotateAngle == tmp.rotateAngle && direction == tmp.direction);
}

const CurveWeight& 
CurveWeight::operator= (const CurveWeight& tmp){
  lp = tmp.lp;
  weight = tmp.weight;
  level = tmp.level;
  midpoint = tmp.midpoint;
  center = tmp.center;
  radius = tmp.radius;
  rotateAngle = tmp.rotateAngle;
  direction = tmp.direction;

  return *this;
}
  
ostream& 
operator<< (ostream& _os, const CurveWeight& w) {
  w.Output(_os);
  return _os;
}

istream& 
operator>> (istream& _is, CurveWeight& w) {
  w.Input(_is);
  return _is;
}
