#include "Body.h"

#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>

#include "MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionMethod.h"

/*---------------------------- Static Initializers ---------------------------*/

string Body::m_modelDataDir;
vector<CollisionDetectionMethod*> Body::m_cdMethods;

/*------------------------------ Construction --------------------------------*/

Body::
Body(MultiBody* _owner) : m_multibody(_owner) { }

/*--------------------------- Metadata Accessors -----------------------------*/

MultiBody*
Body::
GetMultiBody() {
  return m_multibody;
}


const string&
Body::
GetFileName() const {
  return m_filename;
}


string
Body::
GetFilePath() const {
  return m_modelDataDir == "/" || m_filename[0] == '/' ? m_filename :
      m_modelDataDir + m_filename;
}

/*----------------------- Rendering Property Accessors -----------------------*/

bool
Body::
IsColorLoaded() const {
  return m_colorLoaded;
}


const Color4&
Body::
GetColor() const {
  return m_color;
}


bool
Body::
IsTextureLoaded() const {
  return m_textureLoaded;
}


const string&
Body::
GetTexture() const {
  return m_textureFile;
}

/*------------------------- Model Property Accessors -------------------------*/

GMSPolyhedron::COMAdjust
Body::
GetCOMAdjust() const {
  return m_comAdjust;
}


double
Body::
GetMass() const {
  return m_mass;
}


const Matrix3x3&
Body::
GetMoment() const {
  return m_moment;
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


void
Body::
SetPolyhedron(GMSPolyhedron& _poly) {
  m_polyhedron = _poly;
  m_worldPolyhedron = _poly;
  MarkDirty();
}


const GMSPolyhedron&
Body::
GetPolyhedron() const {
  return m_polyhedron;
}


const GMSPolyhedron&
Body::
GetWorldPolyhedron() const {
  if(!m_worldPolyhedronCached)
    ComputeWorldPolyhedron();
  return m_worldPolyhedron;
}


GMSPolyhedron
Body::
GetWorldBoundingBox() const {
  return GetWorldPolyhedron().ComputeBoundingPolyhedron();
}


bool
Body::
IsConvexHullVertex(const Vector3d& _v) const {
  if(!m_convexHullCached)
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
  string filename = GetFilePath();

  if(!FileExists(filename))
    throw ParseException(WHERE, "File \'" + filename + "\' not found.");

  m_polyhedron.Read(filename, _comAdjust);
  m_worldPolyhedron = m_polyhedron;

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
MarkDirty() const {
  m_convexHullCached = false;
  m_transformCached = false;
  m_worldPolyhedronCached = false;
}


void
Body::
ComputeMomentOfInertia() const {
  const Vector3d centerOfMass = m_polyhedron.GetCentroid();
  const vector<Vector3d>& vert = m_polyhedron.m_vertexList;
  const double massPerVert = m_mass / vert.size();
  auto& moment = const_cast<Matrix3x3&>(m_moment);

  moment = Matrix3x3();
  for(const auto& v : vert) {
    Vector3d r = v - centerOfMass;
    moment[0][0] += massPerVert * (r[1] * r[1] + r[2] * r[2]);
    moment[0][1] += massPerVert * -r[0] * r[1];
    moment[0][2] += massPerVert * -r[0] * r[2];
    moment[1][0] += massPerVert * -r[1] * r[0];
    moment[1][1] += massPerVert * (r[0] * r[0] + r[2] * r[2]);
    moment[1][2] += massPerVert * -r[1] * r[2];
    moment[2][0] += massPerVert * -r[0] * r[2];
    moment[2][1] += massPerVert * -r[1] * r[2];
    moment[2][2] += massPerVert * (r[0] * r[0] + r[1] * r[1]);
  }
  moment = inverse(moment);
}


void
Body::
ComputeConvexHull() const {
  using Kernel = GMSPolyhedron::CGALKernel;
  using CGALPolyhedron = CGAL::Polyhedron_3<Kernel>;

  // Compute convex hull of non-collinear points with CGAL.
  const auto& points = m_polyhedron.m_cgalPoints;
  CGALPolyhedron poly;
  CGAL::convex_hull_3(points.begin(), points.end(), poly);

  // Convert from CGAL points to our Vector3d to store the convex hull.
  auto& convexHull = const_cast<GMSPolyhedron&>(m_convexHull);
  for(auto vit = poly.points_begin(); vit != poly.points_end(); ++vit)
    convexHull.m_vertexList.emplace_back(
        to_double((*vit)[0]), to_double((*vit)[1]), to_double((*vit)[2]));

  m_convexHullCached = true;
}


void
Body::
ComputeWorldPolyhedron() const {
  /// @note This method is marked const because it doesn't conceptually alter
  ///       the Body's data - it completes a lazy computation that is initiated
  ///       when we change the world transform and completed when we access
  ///       anything affected by that change.
  auto& poly = const_cast<GMSPolyhedron&>(m_worldPolyhedron);

  using CGAL::to_double;
  using Kernel = GMSPolyhedron::CGALKernel;

  // Get the world transform.
  const auto& transform = GetWorldTransformation();
  const auto& r = transform.rotation().matrix();
  const auto& t = transform.translation();

  // Create a CGAL representation of the world transform.
  CGAL::Aff_transformation_3<Kernel>
      cgalTransform(r[0][0], r[0][1], r[0][2], t[0],
                    r[1][0], r[1][1], r[1][2], t[1],
                    r[2][0], r[2][1], r[2][2], t[2]);

  // Update the vertices.
  const auto& c = m_polyhedron.m_cgalPoints;
  for(size_t i = 0; i < m_polyhedron.m_vertexList.size(); ++i) {
    poly.m_cgalPoints[i] = cgalTransform(c[i]);
    poly.m_vertexList[i] = transform * m_polyhedron.m_vertexList[i];
    /// @TODO Fix CGAL transformation issues and use the same computation for
    ///       transforming both CGAL points and our points.
    //auto x = cgalTransform(c[i]);
    //m_worldPolyhedron.m_vertexList[i](to_double(x[0]), to_double(x[1]),
    //    to_double(x[2]));
  }

  // Update the face normals.
  for(size_t i = 0; i < m_polyhedron.m_polygonList.size(); ++i)
    poly.m_polygonList[i].GetNormal() =
        transform.rotation() * m_polyhedron.m_polygonList[i].GetNormal();

  m_worldPolyhedronCached = true;
}

/*----------------------------------------------------------------------------*/
