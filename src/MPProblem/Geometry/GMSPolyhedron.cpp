#include "GMSPolyhedron.h"
#include "Utilities/MPUtils.h"
#include <fstream>
#include <cstring>
#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"
#include "ModelTool.h"

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
GMSPolyhedron::Read(string _fileName){
  Vector3d com;	//com = Center of Mass

  //---------------------------------------------------------------
  // Get polyhedron file name and try to open the file
  //---------------------------------------------------------------
  ifstream _is(_fileName.c_str());
  if (!_is) {
    cout << "Can't open \"" << _fileName << "\"." << endl;
    exit(1);
  }

  //---------------------------------------------------------------
  // Read polyhedron
  //---------------------------------------------------------------
  string ext = "";
  size_t pos = _fileName.rfind(".");
  if(pos != string::npos)
    ext = _fileName.substr(pos+1);
  if (ext == "dat"){
    com = Read(_is);
  } 
  else if (ext == "g" || ext == "obj"){
    com = ReadModel(_fileName); 
  } 
  else{ 
    cerr << "ERROR: \"" << _fileName << "\" format is unrecognized.";
    cerr << "Formats are recognized by file suffixes: GMS(*.dat), BYU(*.g), and OBJ(*.obj)" << endl;
    exit(1);
  }

  //---------------------------------------------------------------
  // Close file
  //---------------------------------------------------------------
  _is.close();
  return com;
}


//=========================================================================
//  Read
//      reads "original" GMS format
//=========================================================================
Vector3d
GMSPolyhedron::Read(istream& _is){
  Vector3d sum(0,0,0), com;

  int numVertices;
  _is >> numVertices;
  for(int i=0; i<numVertices; ++i){
    Vector3d v;
    _is >> v;
    m_vertexList.push_back(v);
    sum = sum + v;
  }
  com = sum / m_vertexList.size();

  m_maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3d>::iterator V = m_vertexList.begin(); V != m_vertexList.end(); ++V){
    *V = *V - com;
    if(V->norm() > m_maxRadius)
      m_maxRadius = V->norm();
  }

  int numPolygons;
  _is >> numPolygons;
  for(int i=0; i<numPolygons; ++i){
    int numPolyVertices;
    _is >> numPolyVertices;
    GMSPolygon p;
    p.m_vertexList = vector<int>(numPolyVertices, -1);
    for(int j=0; j<numPolyVertices; ++j) 
      _is >> p.m_vertexList[j];
    m_polygonList.push_back(p);
  }

  ComputeNormals();
  return com;
}


//=========================================================================
//  Read
//      reads BYU format
//      OLDer function that takes as parameter istream&
//      new implementation passes file name to CreateModelLoader and returns
//      IModel*
//=========================================================================
Vector3d
GMSPolyhedron::ReadBYU(istream& _is){
  int nParts, numVertices, numPolygons, nEdges;
  _is >> nParts;              // throwaway for now
  _is >> numVertices;
  _is >> numPolygons;
  _is >> nEdges;              // throwaway for now

  int startPartPolys, nPartPolys;
  _is >> startPartPolys;      // throwaway for now
  _is >> nPartPolys;          // throwaway for now

  Vector3d sum(0,0,0), com;
  for(int i=0; i<numVertices; ++i){
    Vector3d v;
    _is >> v;
    m_vertexList.push_back(v);
    sum = sum + v;
  }
  com = sum / m_vertexList.size();

  m_maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3d>::iterator V = m_vertexList.begin(); V != m_vertexList.end(); ++V){
    *V = *V - com;
    if(V->norm() > m_maxRadius)
      m_maxRadius = V->norm();
  }
  com = Vector3d(0.0, 0.0, 0.0);

  for(int i=0; i<numPolygons; ++i){
    GMSPolygon p;
    do{
      int tmp;
      _is >> tmp;
      p.m_vertexList.push_back(tmp);
    } while(p.m_vertexList.back() > 0);
    p.m_vertexList.back() *= -1; //last one is negative, so change sign

    for(vector<int>::iterator I = p.m_vertexList.begin(); I != p.m_vertexList.end(); ++I)
      *I = *I-1; //BYU starts numbering from 1 instead of 0, so decrement by 1

    m_polygonList.push_back(p);
  }

  ComputeNormals();
  return com;
}

//=========================================================================
//  LoadFromIModel
//    loads model using the model loader library
//=========================================================================
void
GMSPolyhedron::LoadFromIModel(IModel* _imodel, Vector3d& _com){
  Vector3d sum(0,0,0);
  typedef IModel::Tri Tri;
  typedef IModel::PtVector PtVector;
  typedef IModel::TriVector TriVector;
  typedef IModel::V3Vcetor V3Vcetor;
  typedef IModel::V2Vcetor V2Vcetor;

  PtVector& verts = _imodel->GetVertices();
  TriVector& tris = _imodel->GetTriP();
  for(int i=0; i<(int)verts.size(); ++i) {
    Vector3d v( verts[i][0], verts[i][1], verts[i][2] );
    m_vertexList.push_back(v);
    sum = sum + v;
  }
  _com = sum / m_vertexList.size();

  m_maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3d>::iterator V = m_vertexList.begin(); V != m_vertexList.end(); ++V){
#ifndef PMPCfgSurface
    *V = *V - _com;
#else
    Vector3d Vtmp = *V;
    Vtmp[0] = Vtmp[0] - _com[0]; //don't mess with height component
    Vtmp[2] = Vtmp[2] - _com[2];
    *V = Vtmp;
#endif

    if(V->norm() > m_maxRadius)
      m_maxRadius = V->norm();
  }

  int numPolygons = (int)tris.size();
  for(int i=0; i<numPolygons; ++i){
    GMSPolygon p;
    p.m_vertexList.push_back(tris[i][0]);
    p.m_vertexList.push_back(tris[i][1]);
    p.m_vertexList.push_back(tris[i][2]);

    m_polygonList.push_back(p);
  }
}

Vector3d
GMSPolyhedron::ReadModel(string _fileName){  
  IModel* imodel = CreateModelLoader(_fileName, false);
  Vector3d com;
  LoadFromIModel( imodel, com );
  ComputeNormals();
  return com;
}

//=========================================================================
//  Write
//=========================================================================
void
GMSPolyhedron::Write(ostream& _os){
  _os << m_vertexList.size() << " " << endl;
  for(vector<Vector3d>::const_iterator V = m_vertexList.begin(); V != m_vertexList.end(); ++V){
    _os << *V;
    _os << endl;
  }
  _os << m_polygonList.size() << " " << endl;
  for(vector<GMSPolygon>::const_iterator P = m_polygonList.begin(); P != m_polygonList.end(); ++P){
    _os << P->m_vertexList.size() << " ";
    for(vector<int>::const_iterator I = P->m_vertexList.begin(); I != P->m_vertexList.end(); ++P)
      _os << *I << " ";
    _os << endl;
  }
  _os << endl;
}

//=========================================================================
//  WriteBYU
//=========================================================================
void
GMSPolyhedron::WriteBYU(ostream& _os){
  _os << "1 " << m_vertexList.size() << " " << m_polygonList.size() << " 1 1 1\n";
  for(vector<Vector3d>::const_iterator V = m_vertexList.begin(); V != m_vertexList.end(); ++V){
    _os << *V;
    _os << endl;
  }
  for(vector<GMSPolygon>::const_iterator P = m_polygonList.begin(); P != m_polygonList.end(); ++P){
    for(vector<int>::const_iterator I = P->m_vertexList.begin(); (I+1) != P->m_vertexList.end(); ++I)
      _os << *I+1 << " ";
    _os << "-" << P->m_vertexList.back()+1 << endl;
  }
  _os << endl;
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

