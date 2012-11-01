#include "Boundary.h"
#include "MPProblem.h"

Boundary::Boundary() {
  /*     cout << "Boundary(). TODO ALL " << endl; */
}

Boundary::Boundary(XMLNodeReader& _node,MPProblem* _problem): MPBaseObject(_node, _problem){ }
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
  if(_par >= (int)m_boundingBox.size())
  {
    cerr << "\n\n\tERROR in Boundary::GetRange(): attempting to get range of parameter " << _par << ", but bounding_box is of size " << m_boundingBox.size() << ", exiting.\n\n";
    exit(-1); 
  }
  return m_boundingBox[_par];
}

void
Boundary::
SetRange(std::vector<double>& _ranges) {
  std::vector<double>::iterator itr;
  int i = 0;
  for (itr = _ranges.begin(); itr < _ranges.end() && i < m_DOFs; itr = itr+2, i++) {
    SetParameter(i,*itr,*(itr+1));
  }
}

void
Boundary::
TranslationalScale(double _scaleFactor) {
  double center, new_first, new_second;
  if (_scaleFactor != 1.0) {
    for (int i = 0; i < m_posDOFs; i++) {
      center = (m_boundingBox[i].first+m_boundingBox[i].second)/2;
      new_first = (m_boundingBox[i].first-center)*_scaleFactor+center;
      new_second = (m_boundingBox[i].second-center)*_scaleFactor+center;
      SetParameter(i,new_first,new_second);
    }
  }
}

Boundary::parameter_type 
Boundary::
GetType(int _par) const {
  return m_parType[_par];
}

int
Boundary::
GetDOFs() const {
  return m_DOFs;//bounding_box.size();
}

int
Boundary::
GetPosDOFs() const {
  return m_posDOFs;//bounding_box.size();
  ///\note This was a bug earlier?  why?  Roger 2008.04.28
}

void
Boundary::
SetParameter(int _par, double _pFirst, double _pSecond) {
  m_boundingBox[_par].first = _pFirst;
  m_boundingBox[_par].second = _pSecond;
}

