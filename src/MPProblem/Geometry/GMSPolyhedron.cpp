// $Id$
///////////////////////////////////////////////////////////////////////////
//  GMSPolyhedron.c
//
//  Created   3/ 6/98 Aaron Michalk
///////////////////////////////////////////////////////////////////////////

#include "GMSPolyhedron.h"
#include "MPUtils.h"
#include <fstream>
#include <cstring>

using namespace std;

///////////////////////////////////////////////////////////////////////////
//Model Loader includes
#include "MovieBYULoader.h"
#include "ModelFactory.h"
#include "ObjLoader.h"
#include "ModelTool.h"

GMSPolygon::GMSPolygon() 
: area(0) 
{}


GMSPolygon::GMSPolygon(const GMSPolygon& _p) 
: vertexList(_p.vertexList), normal(_p.normal), area(_p.area) 
{}


GMSPolygon::~GMSPolygon() 
{}


bool GMSPolygon::operator==(const GMSPolygon& _p) const 
{
  return (vertexList == _p.vertexList) &&
    (normal == _p.normal) &&
    (area == _p.area);
}


GMSPolyhedron::GMSPolyhedron() 
: area(0), maxRadius(0), minRadius(0)
{}


GMSPolyhedron::GMSPolyhedron(const GMSPolyhedron & _p) 
: vertexList(_p.vertexList), polygonList(_p.polygonList), 
  area(_p.area), maxRadius(_p.maxRadius), minRadius(_p.minRadius)
{}


GMSPolyhedron::~GMSPolyhedron() 
{}


bool GMSPolyhedron::operator==(const GMSPolyhedron& _p) const
{
  return (vertexList == _p.vertexList) &&
    (polygonList == _p.polygonList) &&
    (area == _p.area) &&
    (maxRadius == _p.maxRadius) &&
    (minRadius == _p.minRadius);
}


//=========================================================================
//  ComputeNormals
//=========================================================================
void GMSPolyhedron::ComputeNormals() 
{
  double sum = 0;
  for(vector<GMSPolygon>::iterator P = polygonList.begin(); P != polygonList.end(); ++P)
  {
    Vector3D v1 = vertexList[P->vertexList[1]] - vertexList[P->vertexList[0]];
    Vector3D v2 = vertexList[P->vertexList[2]] - vertexList[P->vertexList[0]];
    P->normal = v1.crossProduct(v2);
    P->area = (0.5) * P->normal.magnitude();
    sum += P->area;
    P->normal = P->normal.normalize();
  }
  area = sum;
}


//=========================================================================
//  Read
//      this version of the "Read" method will distinguish which file
//      file format body should request polyhedron to read
//=========================================================================
Vector3D GMSPolyhedron::Read(char* _fileName) 
{
  Vector3D com;	//Center of Mass

  //---------------------------------------------------------------
  // Get polyhedron file name and try to open the file
  //---------------------------------------------------------------
  ifstream _is(_fileName);
  if (!_is) {
    cout << "Can't open \"" << _fileName << "\"." << endl;
    exit(1);
  }

  //---------------------------------------------------------------
  // Read polyhedron
  //---------------------------------------------------------------
  int fileLength = strlen(_fileName);
  if (!strncmp(_fileName+fileLength-4,".dat",4)) 
  {
    com = Read(_is);
  } 
  else if (!strncmp(_fileName+fileLength-2,".g",2) || 
           !strncmp(_fileName+fileLength-4,".obj",4)) 
  {
    com = ReadModel(_fileName); 
  } 
  else 
  {
    cout << "ERROR: \"" << _fileName << "\" format is unrecognized.";
    cout << "\n\n       Formats are recognized by file suffixes:"
      "\n\t    GMS(*.dat)"
      "\n\t    BYU(*.g)"
      "\n\t    OBJ(*.obj)";
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
Vector3D GMSPolyhedron::Read(istream & _is) 
{
  Vector3D sum(0,0,0), com;

  int numVertices;
  _is >> numVertices;
  for(int i=0; i<numVertices; ++i) {
    Vector3D v;
    v.Read(_is);
    vertexList.push_back(v);
    sum = sum + v;
  }
  com = sum / vertexList.size();

  maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3D>::iterator V = vertexList.begin(); V != vertexList.end(); ++V) {
    *V = *V - com;
    if(V->magnitude() > maxRadius)
      maxRadius = V->magnitude();
  }

  int numPolygons;
  _is >> numPolygons;
  for(int i=0; i<numPolygons; ++i) {
    int numPolyVertices;
    _is >> numPolyVertices;
    GMSPolygon p;
    p.vertexList = vector<int>(numPolyVertices, -1);
    for(int j=0; j<numPolyVertices; ++j) 
      _is >> p.vertexList[j];
    polygonList.push_back(p);
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
Vector3D GMSPolyhedron::ReadBYU(istream & _is) 
{
  int nParts, numVertices, numPolygons, nEdges;
  _is >> nParts;              // throwaway for now
  _is >> numVertices;
  _is >> numPolygons;
  _is >> nEdges;              // throwaway for now

  int startPartPolys, nPartPolys;
  _is >> startPartPolys;      // throwaway for now
  _is >> nPartPolys;          // throwaway for now

  Vector3D sum(0,0,0), com;
  for(int i=0; i<numVertices; ++i) {
    Vector3D v;
    v.Read(_is);
    vertexList.push_back(v);
    sum = sum + v;
  }
  com = sum / vertexList.size();

  maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3D>::iterator V = vertexList.begin(); V != vertexList.end(); ++V) {
    *V = *V - com;
    if(V->magnitude() > maxRadius)
      maxRadius = V->magnitude();
  }
  com = Vector3D(0.0, 0.0, 0.0);

  for(int i=0; i<numPolygons; ++i) {
    GMSPolygon p;
    do {
      int tmp;
      _is >> tmp;
      p.vertexList.push_back(tmp);
    } while(p.vertexList.back() > 0);
    p.vertexList.back() *= -1; //last one is negative, so change sign

    for(vector<int>::iterator I = p.vertexList.begin(); I != p.vertexList.end(); ++I)
      *I = *I-1; //BYU starts numbering from 1 instead of 0, so decrement by 1

    polygonList.push_back(p);
  }

  ComputeNormals();
  return com;
}

//=========================================================================
//  LoadFromIModel
//    loads model using the model loader library
//=========================================================================
void GMSPolyhedron::LoadFromIModel(IModel* _imodel, Vector3D& _com) {
  Vector3D sum(0,0,0);
  typedef IModel::Tri Tri;
  typedef IModel::PtVector PtVector;
  typedef IModel::TriVector TriVector;
  typedef IModel::V3Vcetor V3Vcetor;
  typedef IModel::V2Vcetor V2Vcetor;

  PtVector & verts = _imodel->GetVertices();
  TriVector & tris = _imodel->GetTriP();
  for(int i=0; i<(int)verts.size(); ++i) {
    Vector3D v( verts[i][0], verts[i][1], verts[i][2] );
    vertexList.push_back(v);
    sum = sum + v;
  }
  _com = sum / vertexList.size();

  maxRadius = 0.0; // shift center to origin and find maximum radius.
  for(vector<Vector3D>::iterator V = vertexList.begin(); V != vertexList.end(); ++V) {
#ifndef PMPCfgSurface
    *V = *V - _com;
#else
    Vector3D Vtmp = *V;
    Vtmp[0] = Vtmp[0] - _com[0]; //don't mess with height component
    Vtmp[2] = Vtmp[2] - _com[2];
    *V = Vtmp;
#endif

    if(V->magnitude() > maxRadius)
      maxRadius = V->magnitude();
  }

  int numPolygons = (int)tris.size();
  for(int i=0; i<numPolygons; ++i) {
    GMSPolygon p;
    p.vertexList.push_back(tris[i][0]);
    p.vertexList.push_back(tris[i][1]);
    p.vertexList.push_back(tris[i][2]);

    polygonList.push_back(p);
  }
}

Vector3D GMSPolyhedron::ReadModel(char* _fileName) 
{  
  string file(_fileName); 
  IModel* imodel = CreateModelLoader( file, false);
  Vector3D com;
  LoadFromIModel( imodel, com );
  ComputeNormals();
  return com;
}



//=========================================================================
//  Write
//=========================================================================
void GMSPolyhedron::Write(ostream & _os) 
{
  _os << vertexList.size() << " " << endl;
  for(vector<Vector3D>::const_iterator V = vertexList.begin(); V != vertexList.end(); ++V) {
    V->Write(_os);
    _os << endl;
  }
  _os << polygonList.size() << " " << endl;
  for(vector<GMSPolygon>::const_iterator P = polygonList.begin(); P != polygonList.end(); ++P) {
    _os << P->vertexList.size() << " ";
    for(vector<int>::const_iterator I = P->vertexList.begin(); I != P->vertexList.end(); ++P)
      _os << *I << " ";
    _os << endl;
  }
  _os << endl;
}


//=========================================================================
//  WriteBYU
//=========================================================================
void GMSPolyhedron::WriteBYU(ostream & _os) 
{
  _os << "1 " << vertexList.size() << " " << polygonList.size() << " 1 1 1\n";
  for(vector<Vector3D>::const_iterator V = vertexList.begin(); V != vertexList.end(); ++V) {
    V->Write(_os);
    _os << endl;
  }
  for(vector<GMSPolygon>::const_iterator P = polygonList.begin(); P != polygonList.end(); ++P) {
    for(vector<int>::const_iterator I = P->vertexList.begin(); (I+1) != P->vertexList.end(); ++I)
      _os << *I+1 << " ";
    _os << "-" << P->vertexList.back()+1 << endl;
  }
  _os << endl;
}

//=========================================================================
//  GetRandPtOnSurface
//  This function will return a point that lies on the surface of the 
//  polyhedron. Function taken from GB code.
//=========================================================================
Point3d GMSPolyhedron::GetRandPtOnSurface() {
  //using vertices in ptsSurface
  int size=polygonList.size();
  int it;
  bool validIndex=false;
  while( !validIndex ) {
    it=lrand48() % size;
    GMSPolygon& tv=polygonList[it];
    if( tv.vertexList[0]==tv.vertexList[1] || tv.vertexList[1]==tv.vertexList[2] || tv.vertexList[0]==tv.vertexList[2] ) continue;
    else validIndex=true;
  }
  GMSPolygon& tv=polygonList[it];
  double u,v;
  //double offset=0.3;
  u = drand48(); //anything from 0-1 should work for these coords
  v = drand48(); //anything from 0-1 should work for these coords
  if( (u+v)>= 1 ) {
    u = 1-u;
    v = 1-v;
  }
  Vector3d p0 = vertexList[ tv.vertexList[0] ];
  Vector3d p1 = vertexList[ tv.vertexList[1] ];
  Vector3d p2 = vertexList[ tv.vertexList[2] ];
  Vector3d AB = p1 - p0;
  Vector3d AC = p2 - p0;
  Vector3d pt3d = p0 + (u * AB) + (v * AC);
  Point3d rtrnPt(pt3d[0], pt3d[1], pt3d[2]);
  return rtrnPt;
}




///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
/// next two functions adapted from GB code as well 
bool GMSPolyhedron::IsOnSurface(Point2d& _pt, double _h) {
  //using vertices in m_ProjectedPts
  int size=polygonList.size();
  for( int it=0;it<size;it++ ){
    GMSPolygon& poly = polygonList[it];
    if( poly.vertexList[0]==poly.vertexList[1] || poly.vertexList[1]==poly.vertexList[2] || poly.vertexList[0]==poly.vertexList[2] ) continue;
    Vector3D& v0 = vertexList[ poly.vertexList[0] ]; // get vertices of triangle
    Vector3D& v1 = vertexList[ poly.vertexList[1] ];
    Vector3D& v2 = vertexList[ poly.vertexList[2] ];
    Point2d   p0(v0[0], v0[2]);//v0 in xyz, p0 ignores y component
    Point2d   p1(v1[0], v1[2]);//v0 in xyz, p0 ignores y component
    Point2d   p2(v2[0], v2[2]);//v0 in xyz, p0 ignores y component
    if( PtInTriangle( p0, p1, p2, _pt ) ) {
      return true;
    }
  }
  return false;
}

double GMSPolyhedron::HeightAtPt(Point2d _pt, bool& _valid) {
  //using vertices in ptsSurface
  int size=polygonList.size();
  for( int it=0;it<size;it++ ){
    GMSPolygon& poly = polygonList[it];
    if( poly.vertexList[0]==poly.vertexList[1] || poly.vertexList[1]==poly.vertexList[2] || poly.vertexList[0]==poly.vertexList[2] ) continue;
    Vector3D& v0 = vertexList[ poly.vertexList[0] ]; // get vertices of triangle
    Vector3D& v1 = vertexList[ poly.vertexList[1] ];
    Vector3D& v2 = vertexList[ poly.vertexList[2] ];
    Point2d   p0(v0[0], v0[2]);//v0 in xyz, p0 ignores y component
    Point2d   p1(v1[0], v1[2]);//v0 in xyz, p0 ignores y component
    Point2d   p2(v2[0], v2[2]);//v0 in xyz, p0 ignores y component
    Point3d   p03d(v0[0], v0[1], v0[2]);
    Point3d   p13d(v1[0], v1[1], v1[2]);
    Point3d   p23d(v2[0], v2[1], v2[2]);
    double u,v;
    if( PtInTriangle( p0, p1, p2, _pt, u, v ) ) {
      _valid = true;
      Point3d pt3d = GetPtFromBarycentricCoords( p03d, p13d, p23d, u, v ); 
      return pt3d[1];
    }
  }
  _valid = false;
  return -19999.0; //went through all of the triangles and inconsistency found in iscollision check
}
