#include "GMSPolyhedron.h"

#include <fstream>

#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"

#include "Utilities/IOUtils.h"
#include "Utilities/MPUtils.h"

using namespace std;

GMSPolygon::GMSPolygon() : m_area(0){
}

GMSPolygon::GMSPolygon(const GMSPolygon& _p)
  : m_vertexList(_p.m_vertexList), m_normal(_p.m_normal), m_area(_p.m_area){
  }

GMSPolygon::~GMSPolygon() {
}

bool
GMSPolygon::operator==(const GMSPolygon& _p) const{
  return (m_vertexList == _p.m_vertexList) &&
    (m_normal == _p.m_normal) &&
    (m_area == _p.m_area);
}

//Find a unique common vertex between the two polygons
//return -1 if none exists
int
GMSPolygon::CommonVertex(const GMSPolygon& _p) {
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    for(size_t j = 0; j < m_vertexList.size(); ++j) {
      if(m_vertexList[i] == _p.m_vertexList[j]) {
        return m_vertexList[i];
      }
    }
  }
  return -1;
}

pair<int, int>
GMSPolygon::CommonEdge(const GMSPolygon& _p) {
  pair<int, int> edgeID(-1, -1);
  for(size_t i = 0; i < m_vertexList.size(); ++i) {
    for(size_t j = 0; j < m_vertexList.size(); ++j) {
      if(m_vertexList[i] == _p.m_vertexList[j]) {
        if(edgeID.first == -1)
          edgeID.first = m_vertexList[i];
        else
          edgeID.second = m_vertexList[i];
      }
    }
  }
  return edgeID;
}

//End Polygon begin Polyhedron implementation
GMSPolyhedron::GMSPolyhedron(): m_area(0), m_maxRadius(0), m_minRadius(0), m_boundaryBuilt(false), m_force2DBoundary(false){
}

GMSPolyhedron::GMSPolyhedron(const GMSPolyhedron& _p)
  : m_vertexList(_p.m_vertexList), m_polygonList(_p.m_polygonList),
  m_area(_p.m_area), m_maxRadius(_p.m_maxRadius), m_minRadius(_p.m_minRadius),
  m_boundaryLines(_p.m_boundaryLines), m_boundaryBuilt(_p.m_boundaryBuilt),
  m_force2DBoundary(_p.m_force2DBoundary){
}

GMSPolyhedron::~GMSPolyhedron(){
}

bool
GMSPolyhedron::operator==(const GMSPolyhedron& _p) const{
  return (m_vertexList == _p.m_vertexList) &&
    (m_polygonList == _p.m_polygonList) &&
    (m_area == _p.m_area) &&
    (m_maxRadius == _p.m_maxRadius) &&
    (m_minRadius == _p.m_minRadius);
}

//=========================================================================
//  ComputeNormals
//
//  populates Vector3d v1, and Vector3d v2.
//=========================================================================
void
GMSPolyhedron::ComputeNormals(){
  double sum = 0;
  for(vector<GMSPolygon>::iterator P = m_polygonList.begin();P != m_polygonList.end(); ++P){
    Vector3d v1 = m_vertexList[P->m_vertexList[1]] - m_vertexList[P->m_vertexList[0]];
    Vector3d v2 = m_vertexList[P->m_vertexList[2]] - m_vertexList[P->m_vertexList[0]];
    P->m_normal = v1 % v2;
    P->m_area = (0.5) * P->m_normal.norm();
    sum += P->m_area;
    P->m_normal = P->m_normal.normalize();
  }
  m_area = sum;
}


//=========================================================================
//  Read
//      this version of the "Read" method will distinguish which file,
//      file format body should request polyhedron to read
//=========================================================================
Vector3d
GMSPolyhedron::Read(string _fileName, COMAdjust _comAdjust) {
  Vector3d com;	//com = Center of Mass

  if(!FileExists(_fileName))
    throw ParseException(WHERE, "Geometry file '" + _fileName + "' does not exist.");

  //---------------------------------------------------------------
  // Read polyhedron
  //---------------------------------------------------------------
  string ext;
  size_t pos = _fileName.rfind(".");
  if(pos != string::npos)
    ext = _fileName.substr(pos+1);

  if (ext == "g" || ext == "obj") {
    unique_ptr<IModel> imodel(CreateModelLoader(_fileName, false));
    if(!imodel)
      throw ParseException(WHERE, "Cannot read model '" + _fileName + "'.");

    Vector3d com = LoadFromIModel(imodel.get(), _comAdjust);

    ComputeNormals();

    return com;
  }
  else
    throw ParseException(WHERE, _fileName + " has an unrecognized format '" +
        ext + "'. Recognized formats are BYU(.g) and OBJ(.obj).");
}

//=========================================================================
//  LoadFromIModel
//    loads model using the model loader library
//=========================================================================
Vector3d
GMSPolyhedron::LoadFromIModel(IModel* _imodel, COMAdjust _comAdjust) {
  Vector3d com;

  IModel::PtVector& verts = _imodel->GetVertices();
  IModel::TriVector& tris = _imodel->GetTriP();

  for(auto& v : verts) {
    m_vertexList.push_back(v);
    com = com + v;
  }
  com /= m_vertexList.size();

  m_maxRadius = 0; // shift center to origin and find maximum radius.
  for(auto& v : m_vertexList) {

    switch(_comAdjust) {
      case COMAdjust::COM:
        v -= com;
        break;
      case COMAdjust::Surface:
        v[0] -= com[0];
        v[2] -= com[2];
        break;
      case COMAdjust::None:
      default:
        break;
    }

    double dist = v.norm();
    if(dist > m_maxRadius)
      m_maxRadius = dist;
  }

  for(auto& t : tris) {
    GMSPolygon p;
    p.m_vertexList.push_back(t[0]);
    p.m_vertexList.push_back(t[1]);
    p.m_vertexList.push_back(t[2]);
    m_polygonList.push_back(p);
  }

  return com;
}

//=========================================================================
//  Write in BYU format
//=========================================================================
void
GMSPolyhedron::WriteBYU(ostream& _os) {
  size_t numTri = m_polygonList.size();
  _os << "1 " << m_vertexList.size() << " " << numTri << " " << numTri*3 << endl;
  _os << "1 " << numTri << endl;
  for(const auto& v : m_vertexList)
    _os << v << endl;
  for(const auto& p : m_polygonList) {
    for(vector<int>::const_iterator i = p.m_vertexList.begin();
        (i+1) != p.m_vertexList.end(); ++i)
      _os << *i+1 << " ";
    _os << "-" << p.m_vertexList.back()+1 << endl;
  }
}

//=========================================================================
//  GetRandPtOnSurface
//  This function will return a point that lies on the surface of the
//  polyhedron. Function taken from GB code.
//=========================================================================
Point3d
GMSPolyhedron::GetRandPtOnSurface(){
  //using vertices in ptsSurface
  int size=m_polygonList.size();
  int it;
  bool validIndex=false;
  if( DRand() < 0.5 ) { //half the time choose by area proportional to total
    while( !validIndex ){

      double rProp=DRand();//generate a prob from 0..1
      double cummProp=0;
      it=0;
      for(vector<GMSPolygon>::const_iterator P = m_polygonList.begin(); P != m_polygonList.end(); ++P,it++){
	cummProp+= P->m_area/m_area; //this polygon : total area
	if( rProp <= cummProp ) { //reached desired polygon
	  break;
	}
      }
      GMSPolygon& tv=m_polygonList[it];
      if( tv.m_vertexList[0]==tv.m_vertexList[1] || tv.m_vertexList[1]==tv.m_vertexList[2] || tv.m_vertexList[0]==tv.m_vertexList[2] ) continue;
      else validIndex=true;
    }
  }
  else {//randomly select polygon
    while( !validIndex ){
      it=LRand() % size;
      GMSPolygon& tv=m_polygonList[it];
      if( tv.m_vertexList[0]==tv.m_vertexList[1] || tv.m_vertexList[1]==tv.m_vertexList[2] || tv.m_vertexList[0]==tv.m_vertexList[2] ) continue;
      else validIndex=true;
    }
  }
  GMSPolygon& tv=m_polygonList[it];
  double u,v;
  u = DRand(); //anything from 0-1 should work for these coords
  v = DRand(); //anything from 0-1 should work for these coords
  if( (u+v)>= 1 ){
    u = 1-u;
    v = 1-v;
  }
  Vector3d p0 = m_vertexList[ tv.m_vertexList[0] ];
  Vector3d p1 = m_vertexList[ tv.m_vertexList[1] ];
  Vector3d p2 = m_vertexList[ tv.m_vertexList[2] ];
  Vector3d AB = p1 - p0;
  Vector3d AC = p2 - p0;
  Vector3d pt3d = p0 + (u* AB) + (v* AC);
  Point3d rtrnPt(pt3d[0], pt3d[1], pt3d[2]);
  return rtrnPt;
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/// next two functions adapted from GB code as well
bool
GMSPolyhedron::IsOnSurface(Point2d& _pt, double _h){
  //using vertices in m_ProjectedPts
  int size=m_polygonList.size();
  for( int it=0;it<size;it++ ){
    GMSPolygon& poly = m_polygonList[it];
    if( poly.m_vertexList[0]==poly.m_vertexList[1] || poly.m_vertexList[1]==poly.m_vertexList[2] || poly.m_vertexList[0]==poly.m_vertexList[2] ) continue;
    Vector3d& v0 = m_vertexList[ poly.m_vertexList[0] ]; // get vertices of triangle
    Vector3d& v1 = m_vertexList[ poly.m_vertexList[1] ];
    Vector3d& v2 = m_vertexList[ poly.m_vertexList[2] ];
    Point2d   p0(v0[0], v0[2]);//v0 in xyz, p0 ignores y component
    Point2d   p1(v1[0], v1[2]);//v0 in xyz, p0 ignores y component
    Point2d   p2(v2[0], v2[2]);//v0 in xyz, p0 ignores y component
    if( PtInTriangle( p0, p1, p2, _pt ) ){
      return true;
    }
  }
  return false;
}

double
GMSPolyhedron::HeightAtPt(Point2d _pt, bool& _valid){
  //using vertices in ptsSurface
  int size=m_polygonList.size();
  for( int it=0;it<size;it++ ){
    GMSPolygon& poly = m_polygonList[it];
    if( poly.m_vertexList[0]==poly.m_vertexList[1] || poly.m_vertexList[1]==poly.m_vertexList[2] || poly.m_vertexList[0]==poly.m_vertexList[2] ) continue;
    Vector3d& v0 = m_vertexList[ poly.m_vertexList[0] ]; // get vertices of triangle
    Vector3d& v1 = m_vertexList[ poly.m_vertexList[1] ];
    Vector3d& v2 = m_vertexList[ poly.m_vertexList[2] ];
    Point2d   p0(v0[0], v0[2]);//v0 in xyz, p0 ignores y component
    Point2d   p1(v1[0], v1[2]);//v0 in xyz, p0 ignores y component
    Point2d   p2(v2[0], v2[2]);//v0 in xyz, p0 ignores y component
    Point3d   p03d(v0[0], v0[1], v0[2]);
    Point3d   p13d(v1[0], v1[1], v1[2]);
    Point3d   p23d(v2[0], v2[1], v2[2]);
    double u,v;
    if( PtInTriangle( p0, p1, p2, _pt, u, v ) ){
      _valid = true;
      Point3d pt3d = GetPtFromBarycentricCoords( p03d, p13d, p23d, u, v );
      return pt3d[1];
    }
  }
  _valid = false;
  return -19999.0; //went through all of the triangles and inconsistency found in iscollision check
}

void
GMSPolyhedron::BuildBoundary2D() {
  m_boundaryLines.clear();
  m_boundaryBuilt=false;
  m_force2DBoundary=true;
  BuildBoundary();
}

void
GMSPolyhedron::BuildBoundary() {
  if( m_boundaryBuilt ) return; //only allow this to be attempted once
  if( m_boundaryLines.size() > 0 ) return; // this has been done

  m_boundaryBuilt = true;
  typedef vector<GMSPolygon>::iterator PIT;

  //build all the lines locally
  vector< Vector<int,2> > lines;
  //TriVector& triP=m_SurfaceModel->GetTriP();
  //vector<GMSPolygon>& triP = m_polygonList;
  lines.reserve(m_polygonList.size()*3);
  double nearZeroPlane=0.3;
  for(PIT iT=m_polygonList.begin(); iT!=m_polygonList.end();iT++){
    const GMSPolygon& tri=*iT;
    if(m_force2DBoundary) {
      if( fabs( m_vertexList[tri.m_vertexList[0]][1] ) > nearZeroPlane ||
	  fabs( m_vertexList[tri.m_vertexList[1]][1] ) > nearZeroPlane ||
	  fabs( m_vertexList[tri.m_vertexList[2]][1] ) > nearZeroPlane ) continue;
    }
    lines.push_back(Vector<int,2>(tri.m_vertexList[0],tri.m_vertexList[1]));
    lines.push_back(Vector<int,2>(tri.m_vertexList[1],tri.m_vertexList[2]));
    lines.push_back(Vector<int,2>(tri.m_vertexList[2],tri.m_vertexList[0]));
  }

  //Get Boundary lines by checking each line per triangle to see if
  //the line (specified by id) occurs *exactly* once.
  typedef vector< Vector<int,2> >::iterator LIT;
  for(PIT iT=m_polygonList.begin(); iT!=m_polygonList.end();iT++){
    Vector<int,2> line;
    const GMSPolygon& tri=*iT;

    if(m_force2DBoundary) {
      if( fabs( m_vertexList[tri.m_vertexList[0]][1] ) > nearZeroPlane ||
	  fabs( m_vertexList[tri.m_vertexList[1]][1] ) > nearZeroPlane ||
	  fabs( m_vertexList[tri.m_vertexList[2]][1] ) > nearZeroPlane ) continue;
    }

    for( int iD=0; iD<3; iD++ ){
      switch(iD){
	case 0: line(tri.m_vertexList[0],tri.m_vertexList[1]); break;
	case 1: line(tri.m_vertexList[1],tri.m_vertexList[2]); break;
	case 2: line(tri.m_vertexList[2],tri.m_vertexList[0]); break;
      }
      int count=0;

      for( LIT iL=lines.begin(); iL!=lines.end(); iL++ ){
	Vector<int,2>& l=*iL;
	if((l[0]==line[0]&&l[1]==line[1])||(l[0]==line[1]&&l[1]==line[0]))
	  count++;
      }
      if( count==1 )
	m_boundaryLines.push_back( make_pair(line[0],line[1]) );
    }//endfor iD<3
  }//endfor iT
}


////////////////////////////////////////////////////////////////////////////////
//the square of the distance from pos to p1p2
inline double
distsqr3D(const Point3d& _pos, const Point3d& _p1, const Point3d& _p2, Point3d& _cdPt)
{

  Vector3d n=_p1-_p2;
  double t=(n*(_pos-_p1))/(n*(_p2-_p1));
  if( t>=0 && t<=1 ){
    for(int i=0;i<3;i++) _cdPt[i]=(1-t)*_p1[i]+t*_p2[i];
    return (_pos-_cdPt).normsqr();
  }
  else{ //closest point is end pt
    double d1=(_p1-_pos).normsqr();
    double d2=(_p2-_pos).normsqr();
    if( d1<d2 ){ _cdPt=_p1; return d1; }
    else { _cdPt=_p2; return d2; }
  }
}

double GMSPolyhedron::GetClearance(Point3d _pt, Point3d& _closest, int _numRays) {

  double closestPtDist = 1e10;
  Point3d closestPt3d;
  vector< pair<int,int> >& lineIndices = GetBoundaryLines();
  for( int iL=0; iL<(int)lineIndices.size();iL++ ){ //for each BL
    int id1 = m_boundaryLines[iL].first;
    int id2 = m_boundaryLines[iL].second;
    Vector3d vert13dTmp =  m_vertexList[id1];
    Vector3d vert23dTmp =  m_vertexList[id2];

    Point3d edge13d(vert13dTmp[0],vert13dTmp[1],vert13dTmp[2]);
    Point3d edge23d(vert23dTmp[0],vert23dTmp[1],vert23dTmp[2]);

    //find closest pt from p to p1p2
    Point3d c;
    double dist=distsqr3D(_pt,edge13d,edge23d,c);
    if( dist<closestPtDist ) { closestPtDist=dist; closestPt3d=c;}
  }
  _closest = closestPt3d;

  return sqrt(closestPtDist);
}

double GMSPolyhedron::PushToMedialAxis(Point3d& _pt) {
  Point3d orig = _pt;

  Point3d closest;
  int rays = 50;
  double clear=GetClearance(_pt,closest,rays);
  Vector3d dir=(_pt-closest).normalize();
  dir[1]=0;
  dir = dir.normalize()*0.5;
  Point3d newClosest=closest;
  int iteration=0;
  do{
    _pt=_pt+dir;
    Point2d newProjPt(_pt[0],_pt[2]);
    bool stillValid=true;
    double newH = HeightAtPt(newProjPt,stillValid);
    if(!stillValid) {
       _pt=orig;
       break;
    }
    _pt[1]=newH;
    clear=GetClearance(_pt,newClosest,rays);
  }while( (newClosest-closest).normsqr()<0.1 && iteration<1000000 );

  return clear;
}

