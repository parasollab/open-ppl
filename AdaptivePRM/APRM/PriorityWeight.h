/////////////////////////////////////////////////////////////////////
//
//  PriorityWeight.h
//
//  General Description 
//	A derived class of class IWeight. Stores the level at which
//      the edge has already been checked (in addition to the lp,
//      and weight).
//
//  Created
//	07/29/2002 Shawna Thomas
//
//  Last Modified By:
//      xx/xx/xx  <Name>
/////////////////////////////////////////////////////////////////////


#ifndef ProirityWeight_h
#define PriorityWeight_h


#include "Weight.h"
#include <iostream.h>


class PriorityWeight : public IWeight {
public:
  PriorityWeight();

  virtual inline IWeight* clone() const;

  virtual inline int& Level();

  virtual inline void Input(istream& in);
  virtual inline void Output(ostream& out) const;

  bool operator==(const PriorityWeight& tmp) const;
  const PriorityWeight& operator=(const PriorityWeight& tmp);

  friend ostream& operator<<(ostream& _os, const PriorityWeight& w);
  friend istream& operator>>(istream& _is, PriorityWeight& w);
	
protected:
  int level;
};


class PriorityWeightFactory : public DefaulWeightFactory {
  friend class WeightObject;

protected:
  virtual bool Create( IWeight ** ppIWeight /*in/out*/ ) const;
};


#endif
