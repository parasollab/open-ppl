/////////////////////////////////////////////////////////////////////
/********************************************************************
 *@file CurveWeight.h
 *@author Shawna Thomas
 *
 * Weight class for carlike robots
 *
 *@date  6/25/02
 *******************************************************************/

#ifndef CurveWeight_h
#define CurveWeight_h

#include <iostream.h>
#include <vector.h>
#include "Weight.h"
#include "Vectors.h"
#include "Cfg.h"


///////////////////////////////////////////////////////
//
// CurveWeightFactory
//
///////////////////////////////////////////////////////

class CurveWeightFactory : public DefaulWeightFactory {
  friend class CurveWeight;

 protected:
  virtual bool Create( IWeight ** ppIWeight /*in/out*/ ) const;
};


///////////////////////////////////////////////////////
//
// CurveWeight
//
///////////////////////////////////////////////////////

class CurveWeight : public IWeight {

public:

  ///////////////////////////////////////////////////////
  //
  // Constructors and Destructor
  //
  ///////////////////////////////////////////////////////
  /**@name Constructor and Destructor*/
  //@{
  
  /**Default contructor.
    *This constructor sets its data members to invalid value.
    */
  CurveWeight();

  CurveWeight(int _lp, double _weight, const Cfg& _midpoint, const Vector3D& _center, 
	      double _radius, double _rotateAngle, int _direction);

  //@}
  

  virtual inline IWeight* clone() const;

  virtual inline void Input(istream& in);
  virtual inline void Output(ostream& out) const;

  ///////////////////////////////////////////////////////
  //
  // Operator Overloading
  //
  ///////////////////////////////////////////////////////
  /**@name Operator overloading*/
  //@{
  
  ///Compare values from another given CurveWeight instance.
  bool operator== (const CurveWeight& tmp) const;

  ///Copy values from another given CurveWeight instance.
  const CurveWeight & operator= (const CurveWeight& tmp);
  
  ///Output values of datamember to given output stream.
  friend ostream& operator<< (ostream& _os, const CurveWeight& w);

  ///Read values of datamember to given input stream.
  friend istream& operator>> (istream& _is, CurveWeight& w);

  //@}


  ///////////////////////////////////////////////////////
  //
  // Access Methods
  //
  ///////////////////////////////////////////////////////  
  /**@name Access methods*/
  //@{
  
  virtual Cfg& Midpoint(){ return midpoint; }

  virtual Vector3D& Center(){ return center; }

  virtual double& Radius(){ return radius; } 

  virtual double& RotateAngle(){ return rotateAngle; }

  virtual int& Direction(){ return direction; }
  //@}

protected:

  ///////////////////////////////////////////////////////
  //
  // Data
  //
  ///////////////////////////////////////////////////////
  Cfg midpoint;
  Vector3D center;
  double radius;
  double rotateAngle;
  int direction; // 1: forward, -1: backward.
};

#endif
