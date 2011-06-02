// $Id$
// Weight.cpp: implementation for the Weight class.
//
//////////////////////////////////////////////////////////////////////
/*********************************************************************
 *@file Weight.cpp
 *@author Shawna Thomas
 *
 * Weight class for edge weights.  Other weight classes should be 
 * derived off of this class.
 *@date   12/28/02
 *********************************************************************/

#include <Weight.h>
#include <BasicDefns.h>

/////////////////////////////////////////////////////////////
//	Weight
/////////////////////////////////////////////////////////////

double DefaultWeight::MAX_WEIGHT = MAX_DBL;

DefaultWeight::DefaultWeight(){
  lp = INVALID_LP;
  //weight = INVALID_DBL;
  weight = 1;
}

DefaultWeight::DefaultWeight(int lpID){
  lp = lpID;
  //weight = INVALID_DBL;
  weight = 1;
}

DefaultWeight::DefaultWeight(int lpID, double w){
  lp = lpID;
  weight = w;
}

DefaultWeight::~DefaultWeight(){}

double
DefaultWeight::InvalidWeight(){
  return INVALID_DBL;
}

DefaultWeight 
DefaultWeight::MaxWeight(){
  return DefaultWeight(INVALID_LP,MAX_WEIGHT);
}

bool 
DefaultWeight::operator== (const DefaultWeight& tmp) const{
  return ( (lp==tmp.GetLP()) && (weight==tmp.GetWeight()) );
}

const DefaultWeight& 
DefaultWeight::operator= (const DefaultWeight& w){
  lp = w.GetLP();
  weight = w.GetWeight();
  return *this;
}

ostream& operator<< (ostream& _os, const DefaultWeight& w){
  w.Output(_os);
  return _os;
}

void
DefaultWeight::Output(ostream& out) const {
  out << lp << " " << weight;
}

istream& operator>> (istream& _is, DefaultWeight& w){
  w.Input(_is);
  return _is;
}

DefaultWeight 
DefaultWeight::operator+(const DefaultWeight& _other) const {
    return DefaultWeight(weight+_other.weight);
  }
  bool DefaultWeight::operator<(const DefaultWeight& _other) const {
	  return weight < _other.weight;
  }

void
DefaultWeight::Input(istream& in){
  in >> lp >> weight;
}


