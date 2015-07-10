#include "Body.h"

#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>
#include <../src/CGAL/MP_Float.cpp>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include "ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"

string Body::m_modelDataDir;

Body::
Body(MultiBody* _owner) :
  m_multibody(_owner),
  m_label(0),
  m_colorLoaded(false), m_textureLoaded(false),
  m_worldPolyhedronAvailable(false),
  m_convexHullAvailable(false),
  m_centerOfMassAvailable(false) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

Body::
~Body() {
   m_multibody = NULL;
}

Vector3d
Body::
GetCenterOfMass() {
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

GMSPolyhedron&
Body::
GetWorldPolyhedron() {
  if(!m_worldPolyhedronAvailable) {
    typedef vector<Vector3d>::iterator VIT;
    for(size_t i = 0; i < m_polyhedron.m_vertexList.size(); ++i)
      m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * m_polyhedron.m_vertexList[i];
    for(size_t i=0; i<m_polyhedron.m_polygonList.size(); ++i)
      m_worldPolyhedron.m_polygonList[i].m_normal =
        m_worldTransformation.rotation() * m_polyhedron.m_polygonList[i].m_normal;
    m_worldPolyhedronAvailable = true;
  }
  return m_worldPolyhedron;
}

GMSPolyhedron&
Body::
GetWorldBoundingBox() {
  for(size_t i=0; i<m_bbPolyhedron.m_vertexList.size(); ++i)
    m_bbWorldPolyhedron.m_vertexList[i] = m_worldTransformation * m_bbPolyhedron.m_vertexList[i];
  return m_bbWorldPolyhedron;
}

bool
Body::
IsConvexHullVertex(const Vector3d& _v) {
  if(!m_convexHullAvailable)
    ComputeConvexHull();

  for(const auto& vert : m_convexHull.m_vertexList)
    if(_v == vert)
      return true;
  return false;
}

void
Body::
PutWorldTransformation(Transformation& _worldTransformation) {
  m_worldTransformation = _worldTransformation;
}

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
  m_bbPolyhedron.m_vertexList[0] = Vector3d(minx, miny, minz);
  m_bbPolyhedron.m_vertexList[1] = Vector3d(minx, miny, maxz);
  m_bbPolyhedron.m_vertexList[2] = Vector3d(minx, maxy, minz);
  m_bbPolyhedron.m_vertexList[3] = Vector3d(minx, maxy, maxz);
  m_bbPolyhedron.m_vertexList[4] = Vector3d(maxx, miny, minz);
  m_bbPolyhedron.m_vertexList[5] = Vector3d(maxx, miny, maxz);
  m_bbPolyhedron.m_vertexList[6] = Vector3d(maxx, maxy, minz);
  m_bbPolyhedron.m_vertexList[7] = Vector3d(maxx, maxy, maxz);

  FindBoundingBox();
}

void
Body::
ReadOptions(istream& _is, CountingStreamBuffer& _cbs) {
  //read white space
  char c;
  while(isspace(_is.peek()))
    _is.get(c);

  //read '-'
  if(_is.peek() != '-')
    return;
  _is.get(c);

  _is >> c;
  //read optional color
  if(c == 'c') {
    _is >> c; //read c(
    if(c != '(')
      throw ParseException(_cbs.Where(), "Invalid specification of color.");
    m_color = ReadField<Color4>(_is, _cbs, "Invalid specification of color.");
    _is >> c; //read )
    if(c != ')')
      throw ParseException(_cbs.Where(), "Invalid specification of color.");
    m_colorLoaded = true;
  }
  //read optional texture file
  else if(c == 't') {
    _is >> c; //read t(
    if(c != '(')
      throw ParseException(_cbs.Where(), "Invalid specification of texture.");
    m_textureFile = ReadFieldString(_is, _cbs,
        "Invalid specification of texture.", false);
    c = m_textureFile[m_textureFile.length() - 1];
    if(c == ')')
      m_textureFile = m_textureFile.substr(0, m_textureFile.length() - 1);
    else {
      _is >> c; //read )
      if(c != ')')
        throw ParseException(_cbs.Where(), "Invalid specification of texture.");
    }
    m_textureLoaded = true;
  }
  else
    throw ParseException(_cbs.Where(), "Failed reading option '" + ::to_string(c) + "'.");
}

void
Body::ComputeCenterOfMass() {
  GMSPolyhedron& poly = GetWorldPolyhedron();
  m_centerOfMass(0, 0, 0);
  for(const auto& v : poly.m_vertexList)
    m_centerOfMass += v;
  m_centerOfMass /= poly.m_vertexList.size();
  m_centerOfMassAvailable = true;
}

void
Body::
FindBoundingBox() {
  m_worldPolyhedronAvailable = false;
  GMSPolyhedron& poly = GetWorldPolyhedron();
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = numeric_limits<double>::lowest();
  for(const auto& v : poly.m_vertexList) {
    minx = min(minx, v[0]);
    maxx = max(maxx, v[0]);
    miny = min(miny, v[1]);
    maxy = max(maxy, v[1]);
    minz = min(minz, v[2]);
    maxz = max(maxz, v[2]);
  }
  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;
}

void
Body::
ComputeConvexHull() {

  typedef CGAL::Exact_predicates_exact_constructions_kernel  Kernel;
  typedef CGAL::Polyhedron_3<Kernel>                         Polyhedron3;
  typedef Kernel::Point_3                                    Point3;

  //copy polyhedron points into vector
  vector<Point3> points;
  for(const auto& v : m_polyhedron.m_vertexList)
    points.push_back(Point3(v[0], v[1], v[2]));

  //define polyhedron to hold convex hull
  Polyhedron3 poly;

  //compute convex hull of non-collinear points
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  //iterate through convex hull
  for(Polyhedron3::Point_iterator vit = poly.points_begin();
      vit != poly.points_end(); ++vit)
    m_convexHull.m_vertexList.push_back(Vector3d(
          to_double((*vit)[0]), to_double((*vit)[1]), to_double((*vit)[2])
          ));

  m_convexHullAvailable = true;
}
