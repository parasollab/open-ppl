// $Id$
// Weight.cpp: implementation for the Weight class.
//
//////////////////////////////////////////////////////////////////////
/*********************************************************************
 *@file Weight.cpp
 *@author Shawna Thomas
 *
 * Weight class for edge weights.  Other m_weight classes should be 
 * derived off of this class.
 *@date   12/28/02
 *********************************************************************/

#include "CfgTypes.h"
#include <Weight.h>
#include <MPUtils.h>
#include <MetricUtils.h>

/////////////////////////////////////////////////////////////
//	Weight
/////////////////////////////////////////////////////////////

double DefaultWeight::MAX_WEIGHT = MAX_DBL;

DefaultWeight::DefaultWeight(string _lpLabel, double _w, vector<CfgType>& _intermediates):
  m_lpLabel(_lpLabel), m_weight(_w), m_intermediates(_intermediates), m_checkedMult(MAXINT){
  }

DefaultWeight::~DefaultWeight(){}

double
DefaultWeight::InvalidWeight(){
  return INVALID_DBL;
}

DefaultWeight 
DefaultWeight::MaxWeight(){
  return DefaultWeight("INVALID", MAX_WEIGHT);
}

bool 
DefaultWeight::operator==(const DefaultWeight& _tmp) const{
  return ( (m_lpLabel==_tmp.GetLPLabel()) && (m_weight==_tmp.GetWeight()) );
}

const DefaultWeight& 
DefaultWeight::operator=(const DefaultWeight& _w){
  m_lpLabel = _w.GetLPLabel();
  m_weight = _w.GetWeight();
  m_intermediates = _w.GetIntermediates();
  m_checkedMult = _w.GetChecked();
  return *this;
}

ostream& 
operator<<(ostream& _os, const DefaultWeight& _w){
  /*_os << _w.m_intermediates.size() << " ";
  for(vector<CfgType>::const_iterator cit = _w.m_intermediates.begin(); cit!= _w.m_intermediates.end(); cit++){
    _os << *cit << " ";
  }
  */
  //TODO::FIX::for now output 0 for number of intermediates, util vizmo gets updated. Then replace with the above code.
  _os << "0 ";
  _os << _w.m_weight;
  return _os;
}

istream& 
operator>>(istream& _is, DefaultWeight& _w){
  int tmp;
  _is >> tmp >> _w.m_weight;
  return _is;
}

DefaultWeight 
DefaultWeight::operator+(const DefaultWeight& _other) const {
  return DefaultWeight(m_lpLabel, m_weight+_other.m_weight);
}

bool 
DefaultWeight::operator<(const DefaultWeight& _other) const {
  return m_weight < _other.m_weight;
}


