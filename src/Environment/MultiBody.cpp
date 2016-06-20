#include "MultiBody.h"

#include <numeric>

MultiBody::
MultiBody() :
  m_multiBodyType(MultiBodyType::Passive),
  m_radius(0), m_maxAxisRange(0) {
  }

MultiBody::
~MultiBody() {
}

MultiBody::MultiBodyType
MultiBody::
GetMultiBodyTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "PASSIVE")
    return MultiBodyType::Passive;
  else if(_tag == "ACTIVE")
    return MultiBodyType::Active;
#ifdef PMPState
  else if(_tag == "NONHOLONOMIC")
    return MultiBodyType::NonHolonomic;
#endif
  else if(_tag == "SURFACE")
    return MultiBodyType::Surface;
  else if(_tag == "INTERNAL")
    return MultiBodyType::Internal;
  else
    throw ParseException(_where,
        "Unknown MultiBody type '" + _tag + "'."
        " Options are: 'active', 'nonholonomic' 'passive', 'internal', or 'surface'.");
}

string
MultiBody::
GetTagFromMultiBodyType(MultiBodyType _b) {
  switch(_b) {
    case MultiBodyType::Active:
      return "Active";
#ifdef PMPState
    case MultiBodyType::NonHolonomic:
      return "NonHolonomic";
#endif
    case MultiBodyType::Passive:
      return "Passive";
    case MultiBodyType::Internal:
      return "Internal";
    case MultiBodyType::Surface:
      return "Surface";
    default:
      return "Unknown Base Type";
  }
}

void
MultiBody::
BuildCDStructure() {
  for(auto& body : m_bodies)
    body->BuildCDStructure();
}

void
MultiBody::
AddBody(const shared_ptr<Body>& _body) {
  m_bodies.push_back(_body);
}

void
MultiBody::
FindMultiBodyInfo() {
  // Find COM
  m_com(0, 0, 0);
  for(auto& body : m_bodies)
    m_com += body->GetCenterOfMass();
  m_com /= m_bodies.size();

  //Find Bounding box
  double minx, miny, minz, maxx, maxy, maxz;
  minx = miny = minz = numeric_limits<double>::max();
  maxx = maxy = maxz = numeric_limits<double>::lowest();

  for(auto& body : m_bodies) {
    double* tmp = body->GetBoundingBox();
    minx = min(minx, tmp[0]); maxx = max(maxx, tmp[1]);
    miny = min(miny, tmp[2]); maxy = max(maxy, tmp[3]);
    minz = min(minz, tmp[4]); maxz = max(maxz, tmp[5]);
  }

  m_boundingBox[0] = minx; m_boundingBox[1] = maxx;
  m_boundingBox[2] = miny; m_boundingBox[3] = maxy;
  m_boundingBox[4] = minz; m_boundingBox[5] = maxz;

  // Find max axis range
  double rangex = maxx - minx;
  double rangey = maxy - miny;
  double rangez = maxz - minz;
  m_maxAxisRange = max(rangex, max(rangey,rangez));

  // Find bounding radius
  m_radius = m_bodies[0]->GetPolyhedron().m_maxRadius;
  for(size_t i = 1; i < m_bodies.size(); ++i)
    m_radius += m_bodies[i]->GetPolyhedron().m_maxRadius * 2.0;
}

