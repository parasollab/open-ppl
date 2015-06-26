#include "Body.h"

#include <sstream>

#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>
#include <../src/CGAL/MP_Float.cpp>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include "ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"

string Body::m_modelDataDir;

Body::Body(MultiBody* _owner) :
  m_multibody(_owner),
  m_isBase(false),
  m_label(0),
  m_baseType(PLANAR),
  m_baseMovementType(TRANSLATIONAL),
  m_convexHullAvailable(false),
  m_centerOfMassAvailable(false), m_worldPolyhedronAvailable(false) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

Body::Body(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  m_multibody(_owner),
  m_isBase(false),
  m_label(0),
  m_baseType(PLANAR),
  m_baseMovementType(TRANSLATIONAL),
  m_polyhedron(_polyhedron),
  m_worldPolyhedron(_polyhedron),
  m_convexHullAvailable(false),
  m_centerOfMassAvailable(false),
  m_worldPolyhedronAvailable(false) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

Body::~Body() {
   m_multibody=NULL;
}

/*bool
Body::operator==(const Body& _b) const {
  return (m_worldTransformation == _b.m_worldTransformation) &&
    (m_polyhedron == _b.m_polyhedron) &&
    (m_worldPolyhedron == _b.m_worldPolyhedron) &&
    (m_centerOfMass == _b.m_centerOfMass) &&
    (m_boundingBox[0] == _b.m_boundingBox[0]) &&
    (m_boundingBox[1] == _b.m_boundingBox[1]) &&
    (m_boundingBox[2] == _b.m_boundingBox[2]) &&
    (m_boundingBox[3] == _b.m_boundingBox[3]) &&
    (m_boundingBox[4] == _b.m_boundingBox[4]) &&
    (m_boundingBox[5] == _b.m_boundingBox[5]) &&
    (m_bbPolyhedron == _b.m_bbPolyhedron) &&
    (m_bbWorldPolyhedron == _b.m_bbWorldPolyhedron) &&
    (m_forwardConnection == _b.m_forwardConnection) &&
    (m_backwardConnection == _b.m_backwardConnection);
}*/

GMSPolyhedron&
Body::GetWorldPolyhedron() {
  if(!m_worldPolyhedronAvailable) {
    for(size_t i=0; i<m_polyhedron.m_vertexList.size(); ++i)
      m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * m_polyhedron.m_vertexList[i];
    for(size_t i=0; i<m_polyhedron.m_polygonList.size(); ++i)
      m_worldPolyhedron.m_polygonList[i].m_normal = m_worldTransformation.rotation() * m_polyhedron.m_polygonList[i].m_normal;
    m_worldPolyhedronAvailable=true;
  }
  return m_worldPolyhedron;
}

GMSPolyhedron&
Body::GetWorldBoundingBox() {
  for(size_t i=0; i<m_bbPolyhedron.m_vertexList.size(); ++i)
    m_bbWorldPolyhedron.m_vertexList[i] = m_worldTransformation * m_bbPolyhedron.m_vertexList[i];
  return m_bbWorldPolyhedron;
}

Vector3d
Body::GetCenterOfMass(){
  if(!m_centerOfMassAvailable)
    ComputeCenterOfMass();
  return m_centerOfMass;
}

double
Body::
GetBoundingSphereRadius() const {
  return m_polyhedron.m_maxRadius;
}

double
Body::
GetInsideSphereRadius() const {
  return m_polyhedron.m_minRadius;
}

Connection&
Body::GetForwardConnection(size_t _index) {
  if (_index < m_forwardConnection.size())
    return m_forwardConnection[_index];
  else{
    cerr << "Error, in Body::GetForwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

Connection&
Body::GetBackwardConnection(size_t _index) {
  if (_index < m_backwardConnection.size())
    return m_backwardConnection[_index];
  else{
    cerr << "Error, in Body::GetBackwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

void
Body::ChangeWorldPolyhedron() {
  for(size_t i=0; i<m_polyhedron.m_vertexList.size(); i++)  // Transform the vertices
    m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * m_polyhedron.m_vertexList[i];
  for(size_t i=0; i<m_polyhedron.m_polygonList.size(); i++)  // Transform the normals
    m_worldPolyhedron.m_polygonList[i].m_normal = m_worldTransformation.rotation() * m_polyhedron.m_polygonList[i].m_normal;
}

//===================================================================
//  Read
//===================================================================
void
Body::Read() {
  string filename = m_modelDataDir == "/" ? m_filename : m_modelDataDir + m_filename;

  if(!FileExists(filename))
    throw ParseException(filename, "File not found.");

  m_polyhedron.Read(filename);
  m_worldPolyhedron = m_polyhedron;
  GMSPolyhedron poly;
  poly = GetPolyhedron();
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = poly.m_vertexList[0][0];
  miny = maxy = poly.m_vertexList[0][1];
  minz = maxz = poly.m_vertexList[0][2];
  for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
    if(poly.m_vertexList[i][0] < minx)
      minx = poly.m_vertexList[i][0];
    else if(maxx < poly.m_vertexList[i][0])
      maxx = poly.m_vertexList[i][0];

    if(poly.m_vertexList[i][1] < miny)
      miny = poly.m_vertexList[i][1];
    else if(maxy < poly.m_vertexList[i][1])
      maxy = poly.m_vertexList[i][1];

    if(poly.m_vertexList[i][2] < minz)
      minz = poly.m_vertexList[i][2];
    else if(maxz < poly.m_vertexList[i][2])
      maxz = poly.m_vertexList[i][2];
  }

  m_bbPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbWorldPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbPolyhedron.m_vertexList[0] = Vector3d(minx,miny,minz);
  m_bbPolyhedron.m_vertexList[1] = Vector3d(minx,miny,maxz);
  m_bbPolyhedron.m_vertexList[2] = Vector3d(minx,maxy,minz);
  m_bbPolyhedron.m_vertexList[3] = Vector3d(minx,maxy,maxz);
  m_bbPolyhedron.m_vertexList[4] = Vector3d(maxx,miny,minz);
  m_bbPolyhedron.m_vertexList[5] = Vector3d(maxx,miny,maxz);
  m_bbPolyhedron.m_vertexList[6] = Vector3d(maxx,maxy,minz);
  m_bbPolyhedron.m_vertexList[7] = Vector3d(maxx,maxy,maxz);

  FindBoundingBox();
}

void
Body::Write(ostream& _os) {
  static int numBody = 0;
  ostringstream oss;
  oss << "Obj" << numBody++ << ".g";
  _os << oss.str() << " ";
  ofstream ofs(oss.str().c_str());
  m_polyhedron.WriteBYU(ofs);
  ofs.close();
}

void
Body::ComputeCenterOfMass(){
  GMSPolyhedron poly = GetWorldPolyhedron();
  if (poly.m_vertexList.empty()) {
    cout << "\nERROR: No Vertices to take Body::centerOfMass from...\n";
  }
  else{
    Vector3d sum(0,0,0);
    for (size_t i=0; i<poly.m_vertexList.size(); i++) {
      sum = sum + poly.m_vertexList[i];
    }
    m_centerOfMass = sum/poly.m_vertexList.size();
    m_centerOfMassAvailable = true;
  }
}

void
Body::FindBoundingBox(){
  GMSPolyhedron poly;
  m_worldPolyhedronAvailable = false;
  poly = GetWorldPolyhedron();
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = poly.m_vertexList[0][0];
  miny = maxy = poly.m_vertexList[0][1];
  minz = maxz = poly.m_vertexList[0][2];
  for(size_t i = 1 ; i < poly.m_vertexList.size() ; i++){
    if(poly.m_vertexList[i][0] < minx)
      minx = poly.m_vertexList[i][0];
    else if(maxx < poly.m_vertexList[i][0])
      maxx = poly.m_vertexList[i][0];

    if(poly.m_vertexList[i][1] < miny)
      miny = poly.m_vertexList[i][1];
    else if(maxy < poly.m_vertexList[i][1])
      maxy = poly.m_vertexList[i][1];

    if(poly.m_vertexList[i][2] < minz)
      minz = poly.m_vertexList[i][2];
    else if(maxz < poly.m_vertexList[i][2])
      maxz = poly.m_vertexList[i][2];
  }
  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;
}

bool
Body::IsAdjacent(shared_ptr<Body> _otherBody) {
  for(vector<Connection>::iterator C = m_forwardConnection.begin(); C != m_forwardConnection.end(); ++C)
    if(C->GetNextBody() == _otherBody)
      return true;
  for(vector<Connection>::iterator C = m_backwardConnection.begin(); C != m_backwardConnection.end(); ++C)
    if(C->GetPreviousBody() == _otherBody)
      return true;
  return this == _otherBody.get();
}

bool
Body::IsWithinI(shared_ptr<Body> _otherBody, int _i){
  //Visit the recursive, flood-fill based helper function.
  return IsWithinIHelper(this,_otherBody.get(),_i,NULL);
}

bool
Body::IsWithinIHelper(Body* _body1, Body* _body2, int _i, Body* _prevBody){
  if(_body1 == _body2)
    return true;

  if(_i == 0)
    return false;

  typedef vector<Connection>::iterator CIT;
  for(CIT C = _body1->m_forwardConnection.begin(); C != _body1->m_forwardConnection.end(); ++C) {
    Body* next = C->GetNextBody().get();
    if(next != _prevBody && IsWithinIHelper(next, _body2, _i-1, _body1))
      return true;
  }
  for(CIT C =_body1->m_backwardConnection.begin(); C != _body1->m_backwardConnection.end(); ++C) {
    Body* prev = C->GetPreviousBody().get();
    if(prev != _prevBody && IsWithinIHelper(prev, _body2, _i-1, _body1))
      return true;
  }
  return false;
}

////////////////////////////////////////
// Collision Detection Methods
// /////////////////////////////////
void
Body::
BuildCDStructure(CollisionDetectionMethod* _cdMethod) {
  _cdMethod->Build(this);
}

#ifdef USE_SOLID
void
Body::UpdateVertexBase(){
  GMSPolyhedron poly = GetWorldPolyhedron();
  for(size_t q=0; q < poly.m_polygonList.size(); q++) {
    int vertexNum[3];
    for(int i=0; i<3; i++) {
      vertexNum[i] = poly.m_polygonList[q].m_vertexList[i];
      Vector3d &tmp = poly.m_vertexList[vertexNum[i]];
      vertex[3*q+i][0]=tmp[0];
      vertex[3*q+i][1]=tmp[1];
      vertex[3*q+i][2]=tmp[2];
    }
  }
  DT_ChangeVertexBase(base,vertex[0]);
}
#endif

///////////////////////////////////////////////////////////////////////////////
//  Connection Methods
///////////////////////////////////////////////////////////////////////////////
void
Body::Link(const shared_ptr<Body>& _otherBody, const Transformation & _transformationToBody2, const
    DHparameters &_dhparameters, const Transformation & _transformationToDHFrame) {
  Connection c(shared_ptr<Body>(this), _otherBody, _transformationToBody2,
      _dhparameters, _transformationToDHFrame);
  Link(c);
}

void
Body::Link(const Connection& _c) {
  AddForwardConnection(_c);
  _c.GetNextBody()->AddBackwardConnection(_c);
  m_worldPolyhedronAvailable=false;
  m_centerOfMassAvailable=false;
}

bool
Body::IsConvexHullVertex(const Vector3d& _v) {
  if(!m_convexHullAvailable)
    ComputeConvexHull();

  vector<Vector3d>::iterator vit;
  for(vit = m_convexHull.m_vertexList.begin(); vit!= m_convexHull.m_vertexList.end(); ++vit)
    if(_v == *vit)
      return true;
  return false;
}

void
Body::ComputeConvexHull() {

  typedef CGAL::Exact_predicates_exact_constructions_kernel  Kernel;
  typedef CGAL::Polyhedron_3<Kernel>                         Polyhedron3;
  typedef Kernel::Point_3                                    Point3;

  //copy polyhedron points into vector
  vector<Point3> points;
  typedef vector<Vector3d>::iterator VIT;
  for(VIT vit = m_polyhedron.m_vertexList.begin(); vit!= m_polyhedron.m_vertexList.end(); ++vit)
    points.push_back(Point3((*vit)[0], (*vit)[1], (*vit)[2]));

  //define polyhedron to hold convex hull
  Polyhedron3 poly;

  //compute convex hull of non-collinear points
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  //iterate through convex hull
  for(Polyhedron3::Point_iterator vit = poly.points_begin(); vit != poly.points_end(); ++vit)
    m_convexHull.m_vertexList.push_back(Vector3d(to_double((*vit)[0]), to_double((*vit)[1]), to_double((*vit)[2])));

  m_convexHullAvailable = true;
}

Body::Base
Body::
GetBaseFromTag(const string& _tag, const string& _where) {
  if(_tag == "PLANAR")
    return PLANAR;
  else if(_tag == "VOLUMETRIC")
    return VOLUMETRIC;
  else if(_tag == "FIXED")
    return FIXED;
  else if(_tag == "JOINT")
    return JOINT;
  else
    throw ParseException(_where,
        "Unknown base type '" + _tag + "'."
        " Options are: 'planar', 'volumetric', 'fixed', or 'joint'.");
}

Body::BaseMovement
Body::
GetMovementFromTag(const string& _tag, const string& _where) {
  if(_tag == "ROTATIONAL")
    return ROTATIONAL;
  else if (_tag == "TRANSLATIONAL")
    return TRANSLATIONAL;
  else
    throw ParseException(_where,
        "Unknown movement type '" + _tag + "'."
        " Options are: 'rotational' or 'translational'.");
}

string
Body::
GetTagFromBase(const Base& _b) {
  switch(_b) {
    case PLANAR:
      return "PLANAR";
    case VOLUMETRIC:
      return "VOLUMETRIC";
    case FIXED:
      return "FIXED";
    case JOINT:
      return "JOINT";
    default:
      return "Unknown Base Type";
  }
}

string
Body::
GetTagFromMovement(const BaseMovement& _bm) {
  switch(_bm){
    case ROTATIONAL:
      return "ROTATIONAL";
    case TRANSLATIONAL:
      return "TRANSLATIONAL";
    default:
      return "Unknown Base Movement";
  }
}
