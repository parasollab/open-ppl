#include "Boundary.h"
#include "MPProblem.h"

Boundary::Boundary() {
/*     cout << "Boundary(). TODO ALL " << endl; */
}

Boundary::Boundary(XMLNodeReader& in_Node,MPProblem* in_pproblem): MPBaseObject(in_Node, in_pproblem){ }

Boundary::~Boundary() {
/*     cout << "~Boundary(). TODO ALL " << endl; */
}

double
Boundary::GetRandomValueInParameter(int _par) {
  double v=0.0;
  // an orientation angle (not necessarily true for many robots)
  if( _par > (int)m_jointLimits.size() ) { //when par is not valid
    v = DRand(); 
  }
  else { // want something in range of orientation angles
    if(m_jointLimits[_par].first < m_jointLimits[_par].second) { // regularly
      v = m_jointLimits[_par].first + 
	  (m_jointLimits[_par].second - m_jointLimits[_par].first)*DRand();
    }
    else { // check the other way, the angle loops around 1
      v = m_jointLimits[_par].first + 
	  (1-(m_jointLimits[_par].first - m_jointLimits[_par].second))*DRand();
      if(v > 1) 
	v = v - 1;
      else if (v < 0) 
	v = DRand();
      if(v > 1) // only possible if first not in the range of 0-1, wrong
	v = DRand();
    }
  } 
  return v;
}



