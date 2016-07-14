#include "DHParameters.h"
#include "Transformation.h"
#include "Utilities/PMPLExceptions.h"

Transformation
DHParameters::GetTransformation() const {
  Vector3d pos(m_parameterValues[1], -sin(m_parameterValues[0])*m_parameterValues[2],
      cos(m_parameterValues[0])*m_parameterValues[2]);
  Matrix3x3 rot;
  getMatrix3x3(rot,
      cos(m_parameterValues[3]), -sin(m_parameterValues[3]), 0.0,
      sin(m_parameterValues[3])*cos(m_parameterValues[0]), cos(m_parameterValues[3])*cos(m_parameterValues[0]), -sin(m_parameterValues[0]),
      sin(m_parameterValues[3])*sin(m_parameterValues[0]), cos(m_parameterValues[3])*sin(m_parameterValues[0]), cos(m_parameterValues[0]));
  return Transformation(pos, Orientation(rot));
}

istream&
operator>>(istream& _is, DHParameters& _d) {
  string dh[4];

  _is >> dh[0] >> dh[1] >> dh[2] >> dh[3];

  for(size_t i = 0; i < 4; i++) {
    size_t del = dh[i].find(":");
    if(del == string::npos)
      _d.m_parameterRanges[i].first = _d.m_parameterRanges[i].second = atof(dh[i].c_str());
    else {
      _d.m_parameterRanges[i].first = atof((dh[i].substr(0, del)).c_str());
      _d.m_parameterRanges[i].second = atof((dh[i].substr(del+1, dh[i].length())).c_str());
    }
  }

  for(size_t i = 0; i < 4; i++) 
    _d.m_parameterValues[i] = (_d.m_parameterRanges[i].first + _d.m_parameterRanges[i].second)/2;

  return _is;
}

ostream&
operator<<(ostream& _os, const DHParameters& _d) {
  for(size_t i = 0; i < 4; i++) {
    if(_d.m_parameterRanges[i].first == _d.m_parameterRanges[i].second)
      _os << _d.m_parameterValues[i] << " ";
    else
      _os << _d.m_parameterRanges[i].first << ":" << _d.m_parameterRanges[i].second << " ";
  }
  return _os;
}

bool
DHParameters::
IsFixed(size_t _index) const {
  return m_parameterRanges[_index].first == m_parameterRanges[_index].second;
}

void
DHParameters::
SetDHParameters(size_t _index, double _v) {
  m_parameterValues[_index] = _v;
}

bool
DHParameters::
InRange(const vector<double>& _cfg, size_t _dof, size_t _index) const {
  if(_index < 0 || _index >= 4) {
    ostringstream msg;
    msg << "Error: Cannot access DHparameter with index " << _index
      << ". Possible indices are [0, 3]." << endl;
    throw PMPLException("Index out of bound", WHERE, msg.str());
  }
  return _cfg[_dof] > m_parameterRanges[_index].first && _cfg[_dof] < m_parameterRanges[_index].second;
}
