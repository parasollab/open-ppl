#include "Cfg/SSSurfaceMult.h"
#ifdef PMPSSSurfaceMult

SSSurfaceMult::SSSurfaceMult(){}
SSSurfaceMult::SSSurfaceMult(const SSSurfaceMult& _rhs){
  *this = _rhs;
}

void SSSurfaceMult::Read(istream& _is){
  m_cfgs.clear();
  //read in the number of cfgs
  size_t nCfgs = 0;
  _is >> nCfgs;
  for (size_t i=0; i<nCfgs && !_is.fail(); ++i){
    SSSurface nextCfg;
    _is >> nextCfg;
    m_cfgs.push_back(nextCfg);
  }
}

void SSSurfaceMult::Write(ostream& _os) const{
  for (vector<SSSurface>::const_iterator cfgIter = m_cfgs.begin(); cfgIter != m_cfgs.end(); ++cfgIter){
    _os << *cfgIter << endl;
  }
}

ostream&
operator<< (ostream& _os, const SSSurfaceMult& _cfg){
  _cfg.Write(_os);
  return _os;
}

istream&
operator>> (istream& _is, SSSurfaceMult& _cfg){
  _cfg.Read(_is);
  return _is;
}

bool
SSSurfaceMult::ConfigureRobot(Environment* _env) const{
  for(vector<SSSurface>::const_iterator cfgIter = m_cfgs.begin(); cfgIter != m_cfgs.end(); ++cfgIter){
    if (!cfgIter->ConfigureRobot(_env))
      return false;
  }
  return true;
}

void
SSSurfaceMult::SetNumCfgs(size_t _num){
  m_cfgs.clear();
  for(size_t i=0; i<_num; ++i){
    SSSurface next;
    next.SetRobotIndex(i);
    m_cfgs.push_back(next);
  }
}

SSSurfaceMult&
SSSurfaceMult::operator=(const SSSurfaceMult& _rhs){
  m_cfgs = _rhs.m_cfgs;
  return *this;
}

bool
SSSurfaceMult::operator==(const SSSurfaceMult& _cfg) const{
  return m_cfgs == _cfg.m_cfgs;
}

bool
SSSurfaceMult::operator!=(const SSSurfaceMult& _cfg) const{
  return m_cfgs != _cfg.m_cfgs;
}

//arithmetic operators, only dummy implementations for now
SSSurfaceMult
SSSurfaceMult::operator+(const SSSurfaceMult& _cfg) const{
  SSSurfaceMult toReturn = *this;
  return toReturn;
}

SSSurfaceMult&
SSSurfaceMult::operator+=(const SSSurfaceMult& _cfg){
  return *this;
}

SSSurfaceMult
SSSurfaceMult::operator-(const SSSurfaceMult& _cfg) const{
  SSSurfaceMult toReturn = *this;
  return toReturn;
}

SSSurfaceMult&
SSSurfaceMult::operator-=(const SSSurfaceMult& _cfg){
  return *this;
}

SSSurfaceMult SSSurfaceMult::operator-() const{
  SSSurfaceMult toReturn = *this;
  return toReturn;
}

SSSurfaceMult SSSurfaceMult::operator*(double _d) const{
  SSSurfaceMult toReturn = *this;
  return toReturn;
}

SSSurfaceMult& SSSurfaceMult::operator*=(double _d){
  return *this;
}

SSSurfaceMult SSSurfaceMult::operator/(double _d) const{
  SSSurfaceMult toReturn = *this;
  return toReturn;
}

SSSurfaceMult& SSSurfaceMult::operator/=(double _d){
  return *this;
}
#endif
