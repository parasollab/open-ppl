/////////////////////////////////////////////////////////////////////
//
//  PriorityWeight.c
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

#include "PriorityWeight.h"
#include "OBPRM.h"


PriorityWeight::PriorityWeight() {
  lp = INVALID_LP; 
  weight = INVALID_DBL; 
  level = 0;
}

IWeight* 
PriorityWeight::clone() const {
  PriorityWeight *pTmp = new PriorityWeight();

  pTmp->Weight() = weight;
  pTmp->LP() = lp;
  pTmp->Level() = level;

  return pTmp;
}

int& 
PriorityWeight::Level() {
  return level;
}

void
PriorityWeight::Input(istream& in) {
  in >> lp >> weight >> level;
}

void
PriorityWeight::Output(ostream& out) const {
  out << lp << " " << weight << " " << level << " ";
}

bool
PriorityWeight::operator==(const PriorityWeight& tmp) const {
  return (tmp.lp == lp && tmp.weight == weight && tmp.level == level);
}

const PriorityWeight&
PriorityWeight::operator=(const PriorityWeight& tmp) {
  lp = tmp.lp;
  weight = tmp.weight;
  level = tmp.level;

  return *this;
}

ostream& operator<<(ostream& _os, const PriorityWeight& w) {
  w.Output(_os);
  return _os;
}

istream& operator>>(istream& _is, PriorityWeight& w) {
  w.Input(_is);
  return _is;
}


bool 
PriorityWeightFactory::Create( IWeight ** ppIWeight /*in/out*/ ) const {
  //check input
  if( ppIWeight==NULL ) return false;
  *ppIWeight = new PriorityWeight();
  if( *ppIWeight==NULL ) return false; //not enough memory
  return true;
}


