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
  else if(_tag == "NONHOLONOMIC")
    return MultiBodyType::NonHolonomic;
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
    case MultiBodyType::NonHolonomic:
      return "NonHolonomic";
    case MultiBodyType::Passive:
      return "Passive";
    case MultiBodyType::Internal:
      return "Internal";
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
    m_com += body->GetWorldPolyhedron().GetCentroid();
  m_com /= m_bodies.size();

  //Find Bounding box
  double minX, minY, minZ, maxX, maxY, maxZ;
  minX = minY = minZ = numeric_limits<double>::max();
  maxX = maxY = maxZ = numeric_limits<double>::lowest();

  for(auto& body : m_bodies) {
    const auto bbx = body->GetWorldBoundingBox();
    const auto& minVertex = bbx.m_vertexList[0];
    const auto& maxVertex = bbx.m_vertexList[7];

    minX = min(minX, minVertex[0]);
    maxX = max(maxX, maxVertex[1]);
    minY = min(minY, minVertex[2]);
    maxY = max(maxY, maxVertex[3]);
    minZ = min(minZ, minVertex[4]);
    maxZ = max(maxZ, maxVertex[5]);
  }

  m_boundingBox[0] = minX;
  m_boundingBox[1] = maxX;
  m_boundingBox[2] = minY;
  m_boundingBox[3] = maxY;
  m_boundingBox[4] = minZ;
  m_boundingBox[5] = maxZ;

  // Find max axis range
  double rangex = maxX - minX;
  double rangey = maxY - minY;
  double rangez = maxZ - minZ;
  m_maxAxisRange = max(rangex, max(rangey,rangez));

  // Find bounding radius
  m_radius = m_bodies[0]->GetPolyhedron().m_maxRadius;
  for(size_t i = 1; i < m_bodies.size(); ++i)
    m_radius += m_bodies[i]->GetPolyhedron().m_maxRadius * 2.0;
}

