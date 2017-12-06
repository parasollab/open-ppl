#include "MultiBody.h"

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/Connection.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <numeric>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Local Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/// Parse a string into a MultiBody::Type.
/// @param _tag The string to parse.
/// @param _where File location info for error reporting.
/// @return The MultiBody::Type described by _tag.
MultiBody::Type
GetMultiBodyTypeFromTag(std::string _tag, const std::string& _where) {
  // Downcase _tag to make the match case-insensitive.
  std::transform(_tag.begin(), _tag.end(), _tag.begin(), ::tolower);

  if(_tag == "passive")
    return MultiBody::Type::Passive;
  else if(_tag == "active")
    return MultiBody::Type::Active;
  else if(_tag == "internal")
    return MultiBody::Type::Internal;
  else {
    throw ParseException(_where, "Unknown MultiBody type '" + _tag + "'."
        " Options are: 'active', 'nonholonomic' 'passive', or 'internal'.");
    return MultiBody::Type::Passive;
  }
}


/// Print a MultiBody::Type to a string.
/// @param _b The type to print.
/// @return A string representation of _b.
std::string
GetTagFromMultiBodyType(const MultiBody::Type _b) {
  switch(_b) {
    case MultiBody::Type::Active:
      return "Active";
    case MultiBody::Type::Passive:
      return "Passive";
    case MultiBody::Type::Internal:
      return "Internal";
    default:
      return "Unknown Base Type";
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MultiBody ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

MultiBody::
MultiBody(const MultiBody::Type _type)
  : m_multiBodyType(_type)
{ }


MultiBody::
MultiBody(XMLNode& _node) {
  // Read the multibody type.
  const std::string type = _node.Read("type", true, "", "MultiBody type in "
      "{active, passive, internal}");
  m_multiBodyType = GetMultiBodyTypeFromTag(type, _node.Where());

  // Read the free bodies in the multibody node. Each body is either the Base
  // (just one) or a link (any number).
  for(auto& child : _node) {
    if(child.Name() == "Base") {
      // Make sure there is just one base.
      if(m_baseBody)
        throw ParseException(child.Where(), "Only one base is permitted.");

      Body* free = new Body(this, child);
      AddBody(free);
      SetBaseBody(free);
    }
    else if(child.Name() == "Link") {
      // A link node has body information and a single child node describing its
      // connection to its parent body.
      Body* link = new Body(this, child);
      AddBody(link);

      const bool oneChild = std::distance(child.begin(), child.end()) == 1;
      const bool isConnection = child.begin()->Name() == "Connection";
      if(!oneChild or !isConnection)
        throw ParseException(child.Where(), "Link nodes must have exactly one "
            "connection child node.");

      Connection* c = new Connection(this, *child.begin());
      m_joints.push_back(c);
    }
  }

  // Make sure a base was provided.
  if(!m_baseBody)
    throw ParseException(_node.Where(), "MultiBody has no base.");

  SortJoints();
  FindMultiBodyInfo();
}


MultiBody::
MultiBody(const MultiBody& _other)
  : m_multiBodyType(_other.m_multiBodyType),
    m_com(_other.m_com),
    m_radius(_other.m_radius),
    m_boundingBox(_other.m_boundingBox),
    m_maxAxisRange(_other.m_maxAxisRange),
    m_baseIndex(_other.m_baseIndex),
    m_dofInfo(_other.m_dofInfo),
    m_currentDofs(_other.m_currentDofs)
{
  CopyComponentsFrom(_other);
}


MultiBody::
MultiBody(MultiBody&& _other)
  : m_multiBodyType(_other.m_multiBodyType),
    m_com(std::move(_other.m_com)),
    m_radius(_other.m_radius),
    m_boundingBox(std::move(_other.m_boundingBox)),
    m_maxAxisRange(_other.m_maxAxisRange),
    m_baseIndex(_other.m_baseIndex),
    m_dofInfo(std::move(_other.m_dofInfo)),
    m_currentDofs(std::move(_other.m_currentDofs))
{
  MoveComponentsFrom(std::move(_other));
}


MultiBody::
~MultiBody() {
  for(Body* body : m_bodies)
    delete body;
  for(Joint* joint : m_joints)
    delete joint;
}


void
MultiBody::
InitializeDOFs(const Boundary* const _b) {
  m_dofInfo.clear();

  const Range<double> full(-1, 1);
  const DofType position = DofType::Positional,
                rotation = DofType::Rotational;

  // Set the base DOFs. Limit positional values to the boundary's extremes.
  if(m_baseType == Body::Type::Planar) {
    m_dofInfo.emplace_back("Base X Translation ", position, _b->GetRange(0));
    m_dofInfo.emplace_back("Base Y Translation ", position, _b->GetRange(1));

    if(m_baseMovement == Body::MovementType::Rotational)
      m_dofInfo.emplace_back("Base Rotation ", rotation, full);
  }
  else if(m_baseType == Body::Type::Volumetric) {
    m_dofInfo.emplace_back("Base X Translation ", position, _b->GetRange(0));
    m_dofInfo.emplace_back("Base Y Translation ", position, _b->GetRange(1));
    m_dofInfo.emplace_back("Base Z Translation ", position, _b->GetRange(2));

    if(m_baseMovement == Body::MovementType::Rotational) {
      m_dofInfo.emplace_back("Base X Rotation ", rotation, full);
      m_dofInfo.emplace_back("Base Y Rotation ", rotation, full);
      m_dofInfo.emplace_back("Base Z Rotation ", rotation, full);
    }
  }

  for(auto& joint : m_joints) {
    // Make a partial label for this joint out of the body indices.
    const std::string label(std::to_string(joint->GetPreviousBodyIndex()) + "-" +
                            std::to_string(joint->GetNextBodyIndex()));

    switch(joint->GetConnectionType()) {
      case Connection::JointType::Revolute:
        m_dofInfo.emplace_back("Revolute Joint " + label + " Angle",
                               DofType::Joint, joint->GetJointRange(0));
        break;
      case Connection::JointType::Spherical:
        m_dofInfo.emplace_back("Spherical Joint " + label + " Angle 0",
                               DofType::Joint, joint->GetJointRange(0));
        m_dofInfo.emplace_back("Spherical Joint " + label + " Angle 1",
                               DofType::Joint, joint->GetJointRange(1));
        break;
      case Connection::JointType::NonActuated:
        break;
    }
  }

  m_currentDofs.resize(DOF(), 0);
  Configure(m_currentDofs);
  FindMultiBodyInfo();
}

/*-------------------------------- Assignment --------------------------------*/

MultiBody&
MultiBody::
operator=(const MultiBody& _other) {
  m_multiBodyType = _other.m_multiBodyType;
  m_com = _other.m_com;
  m_radius = _other.m_radius;
  m_boundingBox = _other.m_boundingBox;
  m_maxAxisRange = _other.m_maxAxisRange;
  m_baseIndex = _other.m_baseIndex;
  m_dofInfo = _other.m_dofInfo;
  m_currentDofs = _other.m_currentDofs;

  CopyComponentsFrom(_other);

  return *this;
}


MultiBody&
MultiBody::
operator=(MultiBody&& _other) {
  m_multiBodyType = _other.m_multiBodyType;
  m_com = std::move(_other.m_com);
  m_radius = _other.m_radius;
  m_boundingBox = std::move(_other.m_boundingBox);
  m_maxAxisRange = _other.m_maxAxisRange;
  m_baseIndex = _other.m_baseIndex;
  m_dofInfo = std::move(_other.m_dofInfo);
  m_currentDofs = std::move(_other.m_currentDofs);

  MoveComponentsFrom(std::move(_other));

  return *this;
}

/*--------------------------- MultiBody Properties ---------------------------*/

MultiBody::Type
MultiBody::
GetType() const noexcept {
  return m_multiBodyType;
}


bool
MultiBody::
IsActive() const noexcept {
  return m_multiBodyType == Type::Active;
}


bool
MultiBody::
IsPassive() const noexcept {
  return m_multiBodyType != Type::Active;
}


bool
MultiBody::
IsInternal() const noexcept {
  return m_multiBodyType == Type::Internal;
}


size_t
MultiBody::
DOF() const noexcept {
  return m_dofInfo.size();
}


size_t
MultiBody::
PosDOF() const noexcept {
  switch(m_baseType) {
    case Body::Type::Planar:
      return 2;
    case Body::Type::Volumetric:
      return 3;
    default:
      return 0;
  }
}


size_t
MultiBody::
OrientationDOF() const noexcept {
  if(m_baseMovement != Body::MovementType::Rotational)
    return 0;
  else
    return m_baseType == Body::Type::Volumetric ? 3 : 1;
}


size_t
MultiBody::
JointDOF() const noexcept {
  return DOF() - PosDOF() - OrientationDOF();
}


const std::vector<double>&
MultiBody::
GetCurrentDOFs() const noexcept {
  return m_currentDofs;
}

/*------------------------------ Body Accessors ------------------------------*/

size_t
MultiBody::
GetNumBodies() const noexcept {
  return m_bodies.size();
}


Body*
MultiBody::
GetBody(const size_t _i) noexcept {
  return m_bodies[_i];
}


const Body*
MultiBody::
GetBody(const size_t _i) const noexcept {
  return m_bodies[_i];
}


void
MultiBody::
AddBody(Body* const _body) {
  /// @TODO Adjust our body tracking so that we don't have to add bodies in
  ///       index order.
  m_bodies.push_back(_body);
  if(_body->GetIndex() != m_bodies.size() - 1)
    throw ParseException(WHERE, "Added body with index " +
        std::to_string(_body->GetIndex()) + ", but it landed in slot " +
        std::to_string(m_bodies.size()) + ".");
  _body->SetMultiBody(this);
}


void
MultiBody::
SetBaseBody(Body* const _body) {
  m_baseBody = _body;
  m_baseIndex = _body->GetIndex();
  m_baseType = _body->GetBodyType();
  m_baseMovement = _body->GetMovementType();
}


Body::Type
MultiBody::
GetBaseType() const noexcept {
  return m_baseType;
}


Body::MovementType
MultiBody::
GetBaseMovementType() const noexcept {
  return m_baseMovement;
}

/*--------------------------- Geometric Properties ---------------------------*/

const Vector3d&
MultiBody::
GetCenterOfMass() const noexcept {
  if(IsActive())
    throw RunTimeException(WHERE, "There is an error in the center of mass "
        "computation for active multibodies - the COM is not updated as the "
        "object changes configuration. Please correct before using.");
  return m_com;
}


double
MultiBody::
GetBoundingSphereRadius() const noexcept {
  return m_radius;
}


double
MultiBody::
GetMaxAxisRange() const noexcept {
  return m_maxAxisRange;
}


const double*
MultiBody::
GetBoundingBox() const noexcept {
  return m_boundingBox.data();
}


void
MultiBody::
PolygonalApproximation(std::vector<Vector3d>& _result) {
  _result.clear();

  size_t nfree = GetNumBodies();

  //If rigid body then return the line between the first 4 vertices and the
  //second 4 vertices of the world bounding box
  if(nfree == 1) {
    const GMSPolyhedron bbox = GetBody(0)->GetWorldBoundingBox();

    Vector3d joint;
    for(size_t i = 0; i < 4; ++i)
      joint += bbox.m_vertexList[i];
    joint /= 4;
    _result.push_back(joint);

    joint(0, 0, 0);
    for(size_t i = 4; i < 8; ++i)
      joint += bbox.m_vertexList[i];
    joint /= 4;
    _result.push_back(joint);
  }
  else {
    for(size_t i = 0; i < nfree - 1; ++i) {
      const GMSPolyhedron bbox1 = m_bodies[i]->GetWorldBoundingBox();
      const GMSPolyhedron bbox2 = m_bodies[i]->GetForwardConnection(0).
          GetNextBody()->GetWorldBoundingBox();

      //find the four closest pairs of points between bbox1 and bbox2
      const vector<Vector3d>& vertices1 = bbox1.m_vertexList;
      const vector<Vector3d>& vertices2 = bbox2.m_vertexList;
      typedef tuple<size_t, size_t, double> VertexIndexDistance;
      vector<VertexIndexDistance> closestDists;
      for(int num = 0; num < 4; ++num) {
        vector<VertexIndexDistance> distances;
        for(size_t j=0; j < vertices1.size(); ++j)
          for(size_t k=0; k < vertices2.size(); ++k)
            distances.push_back(VertexIndexDistance(j, k,
                  (vertices1[j] - vertices2[k]).norm()));
        closestDists.push_back(*min_element(distances.begin(), distances.end(),
              [](const VertexIndexDistance& _v1, const VertexIndexDistance& _v2) {
              return get<2>(_v1) < get<2>(_v2);
              }));
      }

      //if first body in linkage then
      //add the endpoint of linkage 1 that is not closest to linkage 2
      if(i == 0) {
        Vector3d otherJoint;
        int num = 0;
        for(size_t k = 0; num < 4 && k < bbox1.m_vertexList.size(); ++k)
          if(find_if(closestDists.begin(), closestDists.end(),
                [&](const VertexIndexDistance& _v) {
                return get<0>(_v) == k;
                }) == closestDists.end()) {
            otherJoint += bbox1.m_vertexList[k];
            num++;
          }
        otherJoint /= num;
        _result.push_back(otherJoint);
      }

      //compute the joint as the closest 4 vertices from linkage 1 and linkage 2
      Vector3d joint;
      for(size_t k = 0; k < 4; ++k) {
        joint += bbox1.m_vertexList[get<0>(closestDists[k])];
        joint += bbox2.m_vertexList[get<1>(closestDists[k])];
      }
      joint /= 8;
      _result.push_back(joint);

      //if last body in linkage then
      //add endpoint of linkage 2 that is not closest to linkage 1
      if(i == nfree - 2) {
        Vector3d otherJoint;
        int num = 0;
        for(size_t k = 0; num < 4 && k < bbox2.m_vertexList.size(); ++k)
          if(find_if(closestDists.begin(), closestDists.end(),
                [&](const VertexIndexDistance& _v) {
                return get<1>(_v) == k;
                }) == closestDists.end()) {
            otherJoint += bbox2.m_vertexList[k];
            num++;
          }
        otherJoint = otherJoint / num;

        _result.push_back(otherJoint);
      }
    }
  }
}

/*------------------------------- Connections --------------------------------*/

size_t
MultiBody::
GetNumJoints() const noexcept {
  return m_joints.size();
}


std::vector<MultiBody::Joint*>::const_iterator
MultiBody::
joints_begin() const noexcept {
  return m_joints.begin();
}


std::vector<MultiBody::Joint*>::const_iterator
MultiBody::
joints_end() const noexcept {
  return m_joints.end();
}


const DofType&
MultiBody::
GetDOFType(const size_t _i) const noexcept {
  return m_dofInfo[_i].type;
}


const vector<DofInfo>&
MultiBody::
GetDofInfo() const noexcept {
  return m_dofInfo;
}

/*------------------------- Configuration Methods ----------------------------*/

void
MultiBody::
Configure(const Cfg& _c) {
  Configure(_c.GetData());
}


void
MultiBody::
Configure(const vector<double>& _v) {
  int index = 0;

  std::copy(_v.begin(), _v.begin() + std::min(_v.size(), DOF()),
      m_currentDofs.begin());

  // Configure the base.
  if(m_baseType != Body::Type::Fixed) {
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;
    x = _v[0];
    y = _v[1];
    index += 2;
    if(m_baseType == Body::Type::Volumetric) {
      index++;
      z = _v[2];
    }
    if(m_baseMovement == Body::MovementType::Rotational) {
      if(m_baseType == Body::Type::Planar) {
        ++index;
        gamma = _v[2];
      }
      else {
        index += 3;
        alpha = _v[3];
        beta = _v[4];
        gamma = _v[5];
      }
    }

    Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma * PI,
        beta * PI, alpha * PI)));
    m_baseBody->Configure(t1);
  }

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_bodies[jointIndex]->GetBackwardConnection(0);
    auto& dh = connection.GetDHParameters();

    // Adjust connection to reflect new configuration.
    dh.m_theta = _v[index++] * PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}


void
MultiBody::
Configure(const std::vector<double>& _v, const std::vector<double>& _t) {
  int index = 0, t_index = 0;

  std::copy(_v.begin(), _v.begin() + std::min(_v.size(), DOF()),
      m_currentDofs.begin());

  // Configure the base.
  if(m_baseType != Body::Type::Fixed) {
    double x = 0, y = 0, z = 0, alpha = 0, beta = 0, gamma = 0;

    x = _v[0];
    y = _v[1];
    index += 2;
    if(m_baseType == Body::Type::Volumetric) {
      index++;
      z = _v[2];
    }
    if(m_baseMovement == Body::MovementType::Rotational) {
      if(m_baseType == Body::Type::Planar) {
        ++index;
        gamma = _v[2];
      }
      else {
        index += 3;
        alpha = _v[3];
        beta = _v[4];
        gamma = _v[5];
      }
    }

    Transformation t1(Vector3d(x, y, z), Orientation(EulerAngle(gamma * PI,
        beta * PI, alpha * PI)));
    m_baseBody->Configure(t1);
  }

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object's DHParameters.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_bodies[jointIndex]->GetBackwardConnection(0);
    auto& dh = connection.GetDHParameters();

    // Adjust connection to reflect new configuration.
    m_currentDofs[index++] = _t[t_index]/PI; // Skip the theta value in _v as we will use _t instead.
    dh.m_theta = _t[t_index++];
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}

/*----------------------------------- I/O ------------------------------------*/

void
MultiBody::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  // If not already marked active (as when we are parsing a robot), read the
  // type first.
  if(!IsActive()) {
    const std::string multibodyType = ReadFieldString(_is, _cbs,
        "Failed reading multibody type."
        " Options are: active, passive, internal, or surface.");
    m_multiBodyType = GetMultiBodyTypeFromTag(multibodyType, _cbs.Where());
  }

  // Read passive bodies differently as per the old file format.
  if(IsPassive()) {
    Body* fix = new Body(this);
    fix->Read(_is, _cbs);

    AddBody(fix);

    FindMultiBodyInfo();
    return;
  }

  size_t bodyCount = ReadField<size_t>(_is, _cbs,
      "Failed reading body count.");

  m_baseIndex = -1;
  for(size_t i = 0; i < bodyCount && _is; ++i) {
    //read the free body
    Body* free = new Body(this, i);
    free->Read(_is, _cbs);

    //add object to multibody
    AddBody(free);

    if(free->IsBase() && m_baseIndex == size_t(-1)) {
      m_baseIndex = i;
      m_baseBody = free;
      m_baseType = free->GetBodyType();
      m_baseMovement = free->GetMovementType();
    }
  }

  if(m_baseIndex == size_t(-1))
    throw ParseException(_cbs.Where(), "Active body has no base.");

  //get connection info
  std::string connectionTag = ReadFieldString(_is, _cbs,
      "Failed reading connections tag.");
  if(connectionTag != "CONNECTIONS")
    throw ParseException(_cbs.Where(),
        "Unknwon connections tag '" + connectionTag + "'."
        " Should read 'Connections'.");
  size_t connectionCount = ReadField<size_t>(_is, _cbs,
      "Failed reading number of connections.");

  for(size_t i = 0; i < connectionCount; ++i) {
    //add connection info to multibody connection map
    Connection* c = new Connection(this);
    m_joints.push_back(c);
    m_joints.back()->Read(_is, _cbs);
  }

  SortJoints();
  FindMultiBodyInfo();
}


void
MultiBody::
Write(std::ostream& _os) const {
  // Write type.
  _os << GetTagFromMultiBodyType(m_multiBodyType) << std::endl;

  // If this is a passive body, we write it differently to remain compatible
  // with the old file format.
  if(IsPassive()) {
    for(const auto& body : m_bodies)
      _os << *body << std::endl;
    return;
  }

  // Write free body count and free bodies.
  _os << m_bodies.size() << std::endl;
  for(const auto& body : m_bodies)
    _os << *body << std::endl;

  // Write connections.
  size_t numConnection = 0;
  for(const auto& body : m_bodies)
    numConnection += body->ForwardConnectionCount();

  _os << "Connections\n"
      << numConnection
      << std::endl;

  for(const auto& body : m_bodies)
    for(size_t j = 0; j < body->ForwardConnectionCount(); ++j)
      _os << body->GetForwardConnection(j);
}

/*---------------------------------- Helpers ---------------------------------*/

void
MultiBody::
CopyComponentsFrom(const MultiBody& _other) {
  // Copy the bodies.
  for(const Body* const body : _other.m_bodies)
    AddBody(new Body(*body));

  // Set the base body.
  SetBaseBody(m_bodies[_other.m_baseIndex]);

  // Copy the joints.
  for(const Joint* const joint : _other.m_joints) {
    Joint* const copy = new Joint(*joint);
    copy->SetBodies(this);
    m_joints.push_back(copy);
  }
}


void
MultiBody::
MoveComponentsFrom(MultiBody&& _other) {
  // Move the bodies and set their multibody pointers.
  m_bodies = std::move(_other.m_bodies);
  _other.m_bodies.clear();
  for(Body* const body : m_bodies)
    body->SetMultiBody(this);

  // Set the base body.
  SetBaseBody(m_bodies[_other.m_baseIndex]);

  // Move the joints and update their multibody pointers.
  m_joints = std::move(_other.m_joints);
  _other.m_joints.clear();
  for(Joint* const joint : m_joints)
    joint->SetBodies(this);
}


void
MultiBody::
SortJoints() {
  const static auto sorter =
      [](Connection* const _a, Connection* const _b)
      {
        if(_a->GetPreviousBodyIndex() != _b->GetPreviousBodyIndex())
          return _a->GetPreviousBodyIndex() < _b->GetPreviousBodyIndex();
        return _a->GetNextBodyIndex() < _b->GetNextBodyIndex();
      };
  std::sort(m_joints.begin(), m_joints.end(), sorter);
}


void
MultiBody::
UpdateLinks() {
  for(auto& body : m_bodies)
    if(!body->IsBase())
      body->MarkDirty();

  /// @TODO I think we should be able to remove this forced recomputation and
  ///       let it resolve lazily, need to test though.
  for(auto& body : m_bodies)
    if(body->ForwardConnectionCount() == 0)  // tree tips: leaves.
      body->GetWorldTransformation();
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
  minX = minY = minZ = std::numeric_limits<double>::max();
  maxX = maxY = maxZ = std::numeric_limits<double>::lowest();

  for(auto& body : m_bodies) {
    const auto bbx = body->GetWorldBoundingBox();
    const auto& minVertex = bbx.m_vertexList[0];
    const auto& maxVertex = bbx.m_vertexList[7];

    minX = std::min(minX, minVertex[0]);
    maxX = std::max(maxX, maxVertex[1]);
    minY = std::min(minY, minVertex[2]);
    maxY = std::max(maxY, maxVertex[3]);
    minZ = std::min(minZ, minVertex[4]);
    maxZ = std::max(maxZ, maxVertex[5]);
  }

  m_boundingBox[0] = minX;
  m_boundingBox[1] = maxX;
  m_boundingBox[2] = minY;
  m_boundingBox[3] = maxY;
  m_boundingBox[4] = minZ;
  m_boundingBox[5] = maxZ;

  // Find max axis range
  const double rangex = maxX - minX,
               rangey = maxY - minY,
               rangez = maxZ - minZ;
  m_maxAxisRange = std::max(rangex, std::max(rangey,rangez));

  // Roughly approximate the maximum bounding radius by assuming that all links
  // are chained sequentially end-to-end away from the base.
  m_radius = m_bodies[0]->GetPolyhedron().m_maxRadius;
  for(size_t i = 1; i < m_bodies.size(); ++i)
    m_radius += m_bodies[i]->GetPolyhedron().m_maxRadius * 2.0;
}

/*----------------------------------------------------------------------------*/
