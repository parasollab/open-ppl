#include "MultiBody.h"

#include <numeric>

#include "Cfg/Cfg.h"
#include "MPProblem/Boundary.h"

MultiBody::
MultiBody() :
  m_comAvailable(false), m_bodyType(BodyType::Passive),
  m_maxAxisRange(0) {
    fill(m_boundingBox, m_boundingBox+6, 0);
  }

MultiBody::
~MultiBody() {
}

MultiBody::BodyType
MultiBody::
GetBodyTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "PASSIVE")
    return BodyType::Passive;
  else if(_tag == "ACTIVE")
    return BodyType::Active;
  else if(_tag == "SURFACE")
    return BodyType::Surface;
  else if(_tag == "INTERNAL")
    return BodyType::Internal;
  else
    throw ParseException(_where,
        "Unknown MultiBody type '" + _tag + "'."
        " Options are: 'active', 'passive', 'internal', or 'surface'.");
}

string
MultiBody::GetTagFromBodyType(const BodyType& _b) {
  switch(_b) {
    case BodyType::Active:
      return "Active";
    case BodyType::Passive:
      return "Passive";
    case BodyType::Internal:
      return "Internal";
    case BodyType::Surface:
      return "Surface";
    default:
      return "Unknown Base Type";
  }
}

shared_ptr<Body>
MultiBody::
GetBody(size_t _index) const {
  if(_index < 0 || _index >= m_bodies.size())
    throw RunTimeException(WHERE,
        "Cannot access Body '" + ::to_string(_index) + "'.");
  return m_bodies[_index];
}

size_t
MultiBody::
GetBodyCount() const {
  return m_bodies.size();
}

void
MultiBody::
AddBody(const shared_ptr<Body>& _body) {
  m_bodies.push_back(_body);
}


//-------------------------------------------------------------------
//  GetCenterOfMass
//-------------------------------------------------------------------
Vector3d MultiBody::GetCenterOfMass()
{
  if(!m_comAvailable)
    ComputeCenterOfMass();
  return CenterOfMass;
}

//===================================================================
//  GetMaxAxisRange
//===================================================================
double MultiBody::GetMaxAxisRange() const
{
  return m_maxAxisRange;
}

//===================================================================
//  GetBoundingBox
//===================================================================
const double * MultiBody::GetBoundingBox() const
{
  return m_boundingBox;
}

double
MultiBody::
GetBoundingSphereRadius() const {
  double radius = 0;
  for(auto& body : m_bodies)
    radius += body->GetPolyhedron().m_maxRadius * 2.0;
  return radius;
}

double
MultiBody::
GetInsideSphereRadius() const {
  double radius = numeric_limits<double>::lowest();
  for(auto& body : m_bodies)
    radius = max(body->GetPolyhedron().m_minRadius, radius);
  return radius;
}


void
MultiBody::
BuildCDStructure(CollisionDetectionMethod* _cdMethod) {
  for(auto& body : m_bodies)
    body->BuildCDStructure(_cdMethod);
}

//===================================================================
//  ComputeCenterOfMass
//
//  The degree of approximation in calculating center of mass is
//  the same as in Body.cpp. To be more accurate, we need to
//  modify this function to consider the mass of each body.
//
//===================================================================
void
MultiBody::
ComputeCenterOfMass() {
  Vector3d sum;
  for(auto& body : m_bodies)
    sum += body->GetCenterOfMass();
  CenterOfMass = sum / m_bodies.size();
  m_comAvailable = true;
}


//===================================================================
//  FindBoundingBox
//===================================================================
void MultiBody::FindBoundingBox() {
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = numeric_limits<double>::lowest();

  for(auto& body : m_bodies) {
    body->FindBoundingBox();
    double* tmp = body->GetBoundingBox();
    minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
    miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
    minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
  }

  ///////////////////////////////////////////////////////////
  //Pack
  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;

  ///////////////////////////////////////////////////////////
  // Find m_maxAxisRange
  double rangex = maxx - minx;
  double rangey = maxy - miny;
  double rangez = maxz - minz;
  m_maxAxisRange = max(rangex, max(rangey,rangez));
}


#ifdef USE_SOLID
void MultiBody::UpdateVertexBase() {
  for(vector<shared_ptr<FixedBody> >::iterator I = fixedBody.begin(); I != fixedBody.end(); ++I)
    (*I)->UpdateVertexBase();
  for(vector<shared_ptr<FreeBody> >::iterator I = freeBody.begin(); I != freeBody.end(); ++I)
    (*I)->UpdateVertexBase();
}
#endif


bool
MultiBody::
IsInternal() const{
  return m_bodyType == BodyType::Internal;
}

bool
MultiBody::
IsSurface() const {
  return m_bodyType == BodyType::Surface;
}

bool
MultiBody::
IsActive() const{
  return m_bodyType == BodyType::Active;
}

bool
MultiBody::
IsPassive() const{
  return m_bodyType == BodyType::Passive;
}

