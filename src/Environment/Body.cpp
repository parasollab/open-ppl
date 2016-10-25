#include "Body.h"

#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include "ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"

/*--------------------------- Statics Initializers ----------------------------*/

string Body::m_modelDataDir;
vector<CollisionDetectionMethod*> Body::m_cdMethods;

/*---------------------------- Model Property Accessors ----------------------*/

Vector3d
Body::
GetCenterOfMass() {
  if(!m_centerOfMassAvailable)
    ComputeCenterOfMass();
  return m_centerOfMass;
}


void
Body::
SetPolyhedron(GMSPolyhedron& _poly) {
  m_polyhedron = _poly;
  m_worldPolyhedron = _poly;
  m_centerOfMassAvailable = false;
  m_worldPolyhedronAvailable = false;

  ComputeBoundingPolyhedron();
  ComputeBoundingBox();
}


GMSPolyhedron&
Body::
GetWorldPolyhedron() {
  if(!m_worldPolyhedronAvailable)
    ComputeWorldPolyhedron();
  return m_worldPolyhedron;
}


GMSPolyhedron&
Body::
GetWorldBoundingBox() {
  for(size_t i = 0; i < m_bbPolyhedron.m_vertexList.size(); ++i)
    m_bbWorldPolyhedron.m_vertexList[i] = m_worldTransformation *
        m_bbPolyhedron.m_vertexList[i];
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

/*------------------------------ IO Functions --------------------------------*/

void
Body::
Read(GMSPolyhedron::COMAdjust _comAdjust) {
  string filename = m_modelDataDir == "/" || m_filename[0] == '/' ?
    m_filename : m_modelDataDir + m_filename;

  if(!FileExists(filename))
    throw ParseException(WHERE, "File \'" + filename + "\' not found.");

  m_polyhedron.Read(filename, _comAdjust);
  m_worldPolyhedron = m_polyhedron;

  ComputeBoundingPolyhedron();
  ComputeBoundingBox();
  ComputeMomentOfInertia();
}


void
Body::
ReadOptions(istream& _is, CountingStreamBuffer& _cbs) {
  // Read white space.
  char c;
  while(isspace(_is.peek()))
    _is.get(c);

  while(_is.peek() == '-') {
    // Read '-'.
    _is.get(c);

    // Read next option.
    _is >> c;

    // Parse optional com adjustment.
    if(c == 'a') {
      _is >> c; //read a(
      if(c != '(')
        throw ParseException(_cbs.Where(), "Invalid specification of com "
            "adjustment.");
      string adjust = ReadFieldString(_is, _cbs, "Invalid specification of com "
          "adjustment.");
      c = adjust.back();
      //read )
      if(c != ')')
        throw ParseException(_cbs.Where(), "Invalid specification of com "
            "adjustment.");
      adjust = adjust.substr(0, adjust.size() - 1);
      if(adjust == "COM")
        m_comAdjust = GMSPolyhedron::COMAdjust::COM;
      else if(adjust == "SURFACE")
        m_comAdjust = GMSPolyhedron::COMAdjust::Surface;
      else if (adjust == "NONE")
        m_comAdjust = GMSPolyhedron::COMAdjust::None;
      else
        throw ParseException(_cbs.Where(),
            "Invalid specification of com adjustment: '" + adjust +
            "'. Options are 'COM', 'Surface', or 'None'");
    }
    // Parse color.
    else if(c == 'c') {
      _is >> c; //read c(
      if(c != '(')
        throw ParseException(_cbs.Where(), "Invalid specification of color.");
      m_color = ReadField<Color4>(_is, _cbs, "Invalid specification of color.");
      _is >> c; //read )
      if(c != ')')
        throw ParseException(_cbs.Where(), "Invalid specification of color.");
      m_colorLoaded = true;
    }
    // Parse optional texture file.
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
    // Put back - for possible -x translation.
    else {
      _is.putback(c);
      _is.putback('-');
      return;
    }

    while(isspace(_is.peek()))
      _is.get(c);
  }
}

/*------------------------ Collision Detection Helpers -----------------------*/

void
Body::
BuildCDStructure() {
  for(auto& cd : m_cdMethods)
    cd->Build(this);
}

/*----------------------------- Computation Helpers --------------------------*/

void
Body::
ComputeCenterOfMass() {
  GMSPolyhedron& poly = GetPolyhedron();
  m_centerOfMass(0, 0, 0);
  for(const auto& v : poly.m_vertexList)
    m_centerOfMass += v;
  m_centerOfMass /= poly.m_vertexList.size();
  m_centerOfMassAvailable = true;
}


void
Body::
ComputeMomentOfInertia() {
  Vector3d centerOfMass = GetCenterOfMass();
  vector<Vector3d>& vert = GetPolyhedron().m_vertexList;

  double massPerVert = m_mass / vert.size();
  for(const auto& v : vert) {
    Vector3d r = v - centerOfMass;
    m_moment[0][0] += massPerVert * (r[1]*r[1] + r[2]*r[2]);
    m_moment[0][1] += massPerVert * -r[0] * r[1];
    m_moment[0][2] += massPerVert * -r[0] * r[2];
    m_moment[1][0] += massPerVert * -r[1] * r[0];
    m_moment[1][1] += massPerVert * (r[0]*r[0] + r[2]*r[2]);
    m_moment[1][2] += massPerVert * -r[1] * r[2];
    m_moment[2][0] += massPerVert * -r[0] * r[2];
    m_moment[2][1] += massPerVert * -r[1] * r[2];
    m_moment[2][2] += massPerVert * (r[0]*r[0] + r[1]*r[1]);
  }
  m_moment = inverse(m_moment);
}


void
Body::
ComputeBoundingBox() {
  ///\todo Why does this function force a recomputation of the world polyhedron?
  m_worldPolyhedronAvailable = false;
  double* box = m_boundingBox;
  box[0] = box[2] = box[4] = numeric_limits<double>::max();
  box[1] = box[3] = box[5] = numeric_limits<double>::lowest();

  for(const auto& v : GetWorldPolyhedron().m_vertexList) {
    box[0] = min(box[0], v[0]);
    box[1] = max(box[1], v[0]);
    box[2] = min(box[2], v[1]);
    box[3] = max(box[3], v[1]);
    box[4] = min(box[4], v[2]);
    box[5] = max(box[5], v[2]);
  }
}


void
Body::
ComputeBoundingPolyhedron() {
  const GMSPolyhedron& poly = m_polyhedron;
  double minx, miny, minz, maxx, maxy, maxz;
  minx = maxx = poly.m_vertexList[0][0];
  miny = maxy = poly.m_vertexList[0][1];
  minz = maxz = poly.m_vertexList[0][2];

  for(const auto& v : GetWorldPolyhedron().m_vertexList) {
    minx = min(minx, v[0]);
    maxx = max(maxx, v[0]);
    miny = min(miny, v[1]);
    maxy = max(maxy, v[1]);
    minz = min(minz, v[2]);
    maxz = max(maxz, v[2]);
  }

  m_bbWorldPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbPolyhedron.m_vertexList = vector<Vector3d>(8);
  m_bbPolyhedron.m_vertexList[0] = Vector3d(minx, miny, minz);
  m_bbPolyhedron.m_vertexList[1] = Vector3d(minx, miny, maxz);
  m_bbPolyhedron.m_vertexList[2] = Vector3d(minx, maxy, minz);
  m_bbPolyhedron.m_vertexList[3] = Vector3d(minx, maxy, maxz);
  m_bbPolyhedron.m_vertexList[4] = Vector3d(maxx, miny, minz);
  m_bbPolyhedron.m_vertexList[5] = Vector3d(maxx, miny, maxz);
  m_bbPolyhedron.m_vertexList[6] = Vector3d(maxx, maxy, minz);
  m_bbPolyhedron.m_vertexList[7] = Vector3d(maxx, maxy, maxz);
}


void
Body::
ComputeConvexHull() {
  typedef CGAL::Exact_predicates_exact_constructions_kernel  Kernel;
  typedef CGAL::Polyhedron_3<Kernel>                         Polyhedron3;
  typedef Kernel::Point_3                                    Point3;

  // Copy polyhedron points into CGAL representation.
  vector<Point3> points;
  for(const auto& v : m_polyhedron.m_vertexList)
    points.emplace_back(v[0], v[1], v[2]);

  // Compute convex hull of non-collinear points with CGAL.
  Polyhedron3 poly;
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  // Convert from CGAL points to our Vector3d to store the convex hull.
  for(auto vit = poly.points_begin(); vit != poly.points_end(); ++vit)
    m_convexHull.m_vertexList.emplace_back(
        to_double((*vit)[0]), to_double((*vit)[1]), to_double((*vit)[2]));

  m_convexHullAvailable = true;
}


void
Body::
ComputeWorldPolyhedron() {
  using CGAL::to_double;
  using Kernel = GMSPolyhedron::CGALKernel;

  const auto& r = m_worldTransformation.rotation().matrix();
  const auto& t = m_worldTransformation.translation();

  CGAL::Aff_transformation_3<Kernel> transform(r[0][0], r[0][1], r[0][2], t[0],
                                               r[1][0], r[1][1], r[1][2], t[1],
                                               r[2][0], r[2][1], r[2][2], t[2]);

  const auto& c = m_polyhedron.m_cgalPoints;
  for(size_t i = 0; i < c.size(); ++i)
    m_worldPolyhedron.m_cgalPoints[i] = transform(c[i]);

  auto& vertices = m_polyhedron.m_vertexList;
  for(size_t i = 0; i < vertices.size(); ++i)
    m_worldPolyhedron.m_vertexList[i] = m_worldTransformation * vertices[i];

  auto& polygons = m_polyhedron.m_polygonList;
  for(size_t i = 0; i < polygons.size(); ++i)
    m_worldPolyhedron.m_polygonList[i].GetNormal() =
        m_worldTransformation.rotation() * polygons[i].GetNormal();

  m_worldPolyhedronAvailable = true;
}

/*----------------------------------------------------------------------------*/
