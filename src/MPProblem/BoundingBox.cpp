#include "BoundingBox.h"
#include "Utilities/MPUtils.h"

BoundingBox::BoundingBox() { 
  for(int i = 0; i < 3; ++i)
    m_bbx[i] = make_pair(-numeric_limits<double>::max(), numeric_limits<double>::max());
}

BoundingBox::BoundingBox(const BoundingBox& _bbx) : m_bbx(_bbx.m_bbx) {}

bool
BoundingBox::operator==(const Boundary& _b) const {
  const BoundingBox* bbx = dynamic_cast<const BoundingBox*>(&_b);
  return bbx && m_bbx == bbx->m_bbx;
}

double
BoundingBox::GetMaxDist(double _r1, double _r2) const{
  double maxdist = 0;
  for(int i = 0; i<3; ++i){
    if(m_bbx[i].second != numeric_limits<double>::max()){
      double diff = m_bbx[i].second - m_bbx[i].first;
      maxdist += pow(diff, _r1);
    }
  }
  return pow(maxdist, _r2);
}
    
pair<double, double>
BoundingBox::GetRange(size_t _i) const {
  if(_i > 2){
    cerr << "Error::BoundingBox::GetRange::Invalid access to dimension " << _i << "." << endl;
    exit(1);
  }
  return m_bbx[_i];
}

Point3d
BoundingBox::GetRandomPoint() const {
  Point3d p;
  
  for(int i=0; i<3; ++i)
    p[i] = m_bbx[i].first + (m_bbx[i].second - m_bbx[i].first)*DRand();
  
  return p;
}

bool 
BoundingBox::InBoundary(const Vector3d& _p) const {
  for(int i = 0; i < 3; ++i)
    if( _p[i] < m_bbx[i].first || _p[i] > m_bbx[i].second)
      return false;
  return true;
}

double 
BoundingBox::GetClearance(const Vector3d& _p) const {
  double minClearance = numeric_limits<double>::max();
  for (int i = 0; i < 3; ++i) {
    double clearance = min((_p[i] - m_bbx[i].first ), (m_bbx[i].second - _p[i]));
    if (clearance < minClearance || i == 0)
      minClearance = clearance;
  }
  return minClearance;
}
    
Vector3d
BoundingBox::GetClearancePoint(const Vector3d& _p) const {
  Vector3d clrP = _p;
  double minClearance = numeric_limits<double>::max();
  for (int i = 0; i < 3; ++i) {
    if(_p[i] - m_bbx[i].first < minClearance){
      minClearance = _p[i] - m_bbx[i].first;
      clrP[i] = m_bbx[i].first;
    }
    if(m_bbx[i].second - _p[i] < minClearance){
      minClearance = m_bbx[i].second - _p[i];
      clrP[i] = m_bbx[i].second;
    }
  }
  return clrP;
}

double 
BoundingBox::GetClearance2DSurf(Point2d _pos, Point2d& _cdPt) const {
  double minDist=1e10;
  double cbbx[6]={m_bbx[0].first,m_bbx[0].second,
    m_bbx[1].first,m_bbx[1].second,
    m_bbx[2].first,m_bbx[2].second};
  double dist[4]={_pos[0]-cbbx[0],cbbx[1]-_pos[0],
    _pos[1]-cbbx[4],cbbx[5]-_pos[1]};
  if(dist[0]<minDist){     
    minDist=dist[0];
    _cdPt(cbbx[0], _pos[1]);
  }
  if(dist[1]<minDist){
    minDist=dist[1];
    _cdPt(cbbx[1], _pos[1]);
  }
  if(dist[2]<minDist){
    minDist=dist[2];
    _cdPt(_pos[0], cbbx[4]);
  }
  if(dist[3]<minDist){
    minDist=dist[3];
    _cdPt(_pos[0], cbbx[5]);
  }

  if( minDist<0 ) minDist=0;

  return minDist;
}

void 
BoundingBox::ResetBoundary(vector<pair<double, double> >& _obstBBX, double _d){
  for(int i = 0; i<3; ++i){
    m_bbx[i].first = _obstBBX[i].first - _d;
    m_bbx[i].second = _obstBBX[i].second + _d;
  }
}

void
BoundingBox::Read(istream& _is){
  m_bbx[0].first = m_bbx[1].first = m_bbx[2].first = -numeric_limits<double>::max();
  m_bbx[0].second = m_bbx[1].second = m_bbx[2].second = numeric_limits<double>::max();

  //read next three tokens
  string line;
  getline(_is, line);
  istringstream iss(line);
  for(size_t i = 0; i<3; ++i){
    string tok;
    if(iss >> tok){
      size_t del = tok.find(":");
      if(del == string::npos){
        cerr << "Error::Reading bounding box range " << i << ". Should be delimited by ':'." << endl;
        exit(1);
      }
      istringstream minv(tok.substr(0,del)), maxv(tok.substr(del+1, tok.length()));
      if(!(minv>>m_bbx[i].first && maxv>>m_bbx[i].second)){
        cerr << "Error::Reading bounding box range " << i << "." << endl;
        exit(1);
      }
    }
    else if(i<2) { //error. only 1 token provided.
      cerr << "Error::Reading bounding box ranges. Only one provided." << endl;
      exit(1);
    }
  }
}

void
BoundingBox::Write(ostream& _os) const {
  _os << "[ ";
  _os << m_bbx[0].first << ':' << m_bbx[0].second << " ; ";
  _os << m_bbx[1].first << ':' << m_bbx[1].second;
  if(m_bbx[2].second != numeric_limits<double>::max())
    _os << " ; " << m_bbx[2].first << ':' << m_bbx[2].second;
  _os << " ]";
}

