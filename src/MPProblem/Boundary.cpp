#include "Boundary.h"
#include "MPProblem.h"

Boundary::Boundary() {
/*     cout << "Boundary(). TODO ALL " << endl; */
}

Boundary::Boundary(XMLNodeReader& in_Node,MPProblem* in_pproblem): MPBaseObject(in_Node, in_pproblem){ }
///Empty destructor- compiler knows how to do this
Boundary::~Boundary() {
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

const std::pair<double,double>
Boundary::
GetRange(int _par) const {
  if(_par >= (int)bounding_box.size())
          {
                cerr << "\n\n\tERROR in Boundary::GetRange(): attempting to get range of parameter " << _par << ", but bounding_box is of size " << bounding_box.size() << ", exiting.\n\n";
          exit(-1);
          }
   return bounding_box[_par];
  }

void
Boundary::
SetRange(std::vector<double> &_ranges) {
 std::vector<double>::iterator itr;
  int i = 0;
  for (itr = _ranges.begin(); itr < _ranges.end() && i < dofs; itr = itr+2, i++) {
    SetParameter(i,*itr,*(itr+1));
    }
}

void
Boundary::
TranslationalScale(double _scaleFactor) {
 double center, new_first, new_second;
 if (_scaleFactor != 1.0) {
   for (int i = 0; i < pos_dofs; i++) {
     center = (bounding_box[i].first+bounding_box[i].second)/2;
     new_first = (bounding_box[i].first-center)*_scaleFactor+center;
     new_second = (bounding_box[i].second-center)*_scaleFactor+center;
     SetParameter(i,new_first,new_second);
   }
 }
}


int
Boundary::
GetDOFs() const {
    return dofs;//bounding_box.size();
}

int
Boundary::
GetPosDOFs() const {
    return pos_dofs;//bounding_box.size();
      ///\note This was a bug earlier?  why?  Roger 2008.04.28
}

void
Boundary::
SetParameter(int _par, double _pFirst, double _pSecond) {
   bounding_box[_par].first = _pFirst;
   bounding_box[_par].second = _pSecond;
}

