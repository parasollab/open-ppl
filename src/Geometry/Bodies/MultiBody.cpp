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

      m_baseIndex = AddBody(Body(this, child));
    }
    else if(child.Name() == "Link") {
      // A link node has body information and a single child node describing its
      // connection to its parent body.
      AddBody(Body(this, child));

      const bool oneChild = std::distance(child.begin(), child.end()) == 1;
      const bool isConnection = child.begin()->Name() == "Connection";
      if(!oneChild or !isConnection)
        throw ParseException(child.Where(), "Link nodes must have exactly one "
            "connection child node. Support for multiple connections is not "
            "yet implemented.");

      m_joints.emplace_back(new Connection(this, *child.begin()));
    }
  }
  SetBaseBody(m_baseIndex);

  for(auto& joint : m_joints)
    joint->SetBodies();

  // Make sure a base was provided.
  if(!m_baseBody)
    throw ParseException(_node.Where(), "MultiBody has no base.");

  SortJoints();
  FindMultiBodyInfo();
}


MultiBody::
MultiBody(const MultiBody& _other) {
  *this = _other;
}


MultiBody::
MultiBody(MultiBody&& _other) {
  *this = std::move(_other);
}


MultiBody::
~MultiBody() = default;


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

  if(IsComposite())
    throw RunTimeException(WHERE, "Any composite bodies should be handled "
                                  "through group cfgs now.");

  m_currentDofs.resize(DOF(), 0);
  Configure(m_currentDofs);
  FindMultiBodyInfo();
}

/*-------------------------------- Assignment --------------------------------*/

MultiBody&
MultiBody::
operator=(const MultiBody& _other) {
  m_multiBodyType = _other.m_multiBodyType;
  m_com           = _other.m_com;
  m_radius        = _other.m_radius;
  m_boundingBox   = _other.m_boundingBox;
  m_maxAxisRange  = _other.m_maxAxisRange;
  m_baseIndex     = _other.m_baseIndex;
  m_dofInfo       = _other.m_dofInfo;
  m_currentDofs   = _other.m_currentDofs;

  // Copy the bodies.
  for(const auto& body : _other.m_bodies)
    AddBody(Body(body));

  // Set the base body.
  SetBaseBody(m_baseIndex);

  // Copy the joints.
  for(const auto& joint : _other.m_joints) {
    m_joints.emplace_back(new Connection(*joint));
    m_joints.back()->SetBodies(this);
  }

  return *this;
}


MultiBody&
MultiBody::
operator=(MultiBody&& _other) {
  m_multiBodyType = std::move(_other.m_multiBodyType);
  m_com           = std::move(_other.m_com);
  m_radius        = std::move(_other.m_radius);
  m_boundingBox   = std::move(_other.m_boundingBox);
  m_maxAxisRange  = std::move(_other.m_maxAxisRange);
  m_baseIndex     = std::move(_other.m_baseIndex);
  m_dofInfo       = std::move(_other.m_dofInfo);
  m_currentDofs   = std::move(_other.m_currentDofs);

  // Move the bodies and set their multibody pointers.
  m_bodies = std::move(_other.m_bodies);
  _other.m_bodies.clear();
  for(auto& body : m_bodies)
    body.SetMultiBody(this);

  // Set the base body.
  SetBaseBody(m_baseIndex);

  // Move the joints and update their multibody pointers.
  m_joints = std::move(_other.m_joints);
  _other.m_joints.clear();
  for(auto& joint : m_joints)
    joint->SetBodies(this);

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


bool
MultiBody::
IsComposite() const noexcept {
  return m_joints.empty() and m_bodies.size() > 1;
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


std::vector<double>
MultiBody::
GetCurrentCfg() noexcept {
  std::vector<double> output(DOF(), 0);

  // First get the DOFs for the base.
  const std::vector<double> base = GetBase()->GetWorldTransformation().GetCfg();

  // Define the boundaries of each DOF segment.
  auto pos = output.begin(),
       ori = output.begin() + PosDOF(),
       jnt = output.begin() + PosDOF() + OrientationDOF(),
       end = output.end();

  // Determine how many orientation values from 'base' will be unused.
  const size_t ignore = 3 - OrientationDOF();

  // Copy the required values into the output vector, depending on base movement
  // type. For the orientation, we will skip the unused values.
  std::copy(base.begin(), base.begin() + PosDOF(), pos);
  std::copy(base.begin() + 3 + ignore, base.end(), ori);

  // Make sure the orientation values are normalized.
  for(auto iter = ori; iter < jnt; ++iter)
    *iter = Normalize(*iter);

  // Make sure the joint transforms are current.
  UpdateLinks();

  // For each joint, copy its values.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object's DHParameters.
    const DHParameters& dh = joint->GetDHParameters();

    // Set the joint DOF values from the DH params.
    *jnt++ = dh.m_theta / PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      *jnt++ = dh.m_alpha / PI;
  }

  // If jnt is not at the end now, we did something wrong.
  if(jnt != end)
    throw RunTimeException(WHERE) << "Computation error:"
                                  << "\n\tDOF: " << DOF()
                                  << "\n\tEnd distance: " << std::distance(pos, end)
                                  << "\n\tJnt distance: " << std::distance(pos, jnt)
                                  << std::endl;

  m_currentDofs = output;
  return output;
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
  return &m_bodies[_i];
}


const Body*
MultiBody::
GetBody(const size_t _i) const noexcept {
  return &m_bodies[_i];
}


size_t
MultiBody::
AddBody(Body&& _body) {
  /// @TODO Adjust our body tracking so that we don't have to add bodies in
  ///       index order.
  m_bodies.push_back(std::move(_body));
  auto& body = m_bodies.back();
  if(body.GetIndex() != m_bodies.size() - 1)
    throw ParseException(WHERE, "Added body with index " +
        std::to_string(body.GetIndex()) + ", but it landed in slot " +
        std::to_string(m_bodies.size()) + ".");
  body.SetMultiBody(this);

  return body.GetIndex();
}


Body*
MultiBody::
GetBase() noexcept {
  return m_baseBody;
}


const Body*
MultiBody::
GetBase() const noexcept {
  return m_baseBody;
}


void
MultiBody::
SetBaseBody(const size_t _index) {
  if(_index >= m_bodies.size())
    throw RunTimeException(WHERE, "Cannot use index " + std::to_string(_index) +
        " for base body because there are only " +
        std::to_string(m_bodies.size()) + " parts in this multibody.");

  m_baseIndex    = _index;
  m_baseBody     = GetBody(_index);
  m_baseType     = m_baseBody->GetBodyType();
  m_baseMovement = m_baseBody->GetMovementType();
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
    ///@TODO This needs to be fixed to go through all of each obstacle's bodies.
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
      const GMSPolyhedron bbox1 = m_bodies[i].GetWorldBoundingBox();
      const GMSPolyhedron bbox2 = m_bodies[i].GetForwardConnection(0).
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

const std::vector<std::unique_ptr<Connection>>&
MultiBody::
GetJoints() const noexcept {
  return m_joints;
}


Connection*
MultiBody::
GetJoint(const size_t _i) noexcept {
  return m_joints[_i].get();
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


void
MultiBody::
UpdateJointLimits() noexcept {
  const size_t firstJointIndex = PosDOF() + OrientationDOF();

  auto joint = m_joints.begin(); // iter to unique_ptr
  for(size_t i = firstJointIndex; i < DOF(); ++i) {
    switch(joint->get()->GetConnectionType())
    {
      case Connection::JointType::Revolute:
        m_dofInfo[i].range = joint->get()->GetJointRange(0);
        break;
      case Connection::JointType::Spherical:
        m_dofInfo[i].range   = joint->get()->GetJointRange(0);
        m_dofInfo[++i].range = (++joint)->get()->GetJointRange(1);
        break;
      case Connection::JointType::NonActuated:
        break;
    }
    ++joint;
  }
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
    Transformation t1 = GenerateModelTransformation(_v, index, m_baseMovement,
                                                    m_baseType);
    m_baseBody->Configure(t1);
  }

  if(IsComposite())
    throw RunTimeException(WHERE) << "Composite bodies should be handled "
                                  << "through group cfgs.";

  // Configure the links.
  for(auto& joint : m_joints) {
    // Skip non-actuated joints.
    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    // Get the connection object.
    const size_t jointIndex = joint->GetNextBodyIndex();
    auto& connection = m_bodies[jointIndex].GetBackwardConnection(0);
    auto& dh = connection.GetDHParameters();

    // Adjust connection to reflect new configuration.
    dh.m_theta = _v[index++] * PI;
    if(joint->GetConnectionType() == Connection::JointType::Spherical)
      dh.m_alpha = _v[index++] * PI;
  }

  // The base transform has been updated, now update the links.
  UpdateLinks();
}


Transformation
MultiBody::
GenerateModelTransformation(const std::vector<double>& _v, int& _index,
                            const Body::MovementType _movementType,
                            const Body::Type _bodyType) const {
  double z = 0, alpha = 0, beta = 0, gamma = 0;
  const double x = _v[_index];
  const double y = _v[++_index];
  if(_bodyType == Body::Type::Volumetric)
    z = _v[++_index];

  if(_movementType == Body::MovementType::Rotational) {
    if(_bodyType == Body::Type::Planar)
      alpha = _v[++_index]; // Rotation about Z
    else {
      gamma = _v[++_index]; // Rotation about X is the third Euler angle
      beta  = _v[++_index]; // Rotation about Y is the second Euler angle
      alpha = _v[++_index]; // Rotation about Z is the first Euler angle
    }
  }
  ++_index; // Needs to happen to make up for first indexing with no addition.

  // Generate the transformation with a translation vector and ZYX Euler angle.
  return Transformation(Vector3d(x, y, z),
                        EulerAngle(alpha * PI, beta * PI, gamma * PI));
}


void
MultiBody::
PushToNearestValidConfiguration() {
  // First get the actual configured Cfg.
  std::vector<double> cfg = GetCurrentCfg();

  // Check each value against the joint limits.
  for(size_t i = 0; i < DOF(); ++i) {
    const Range<double>& limit = m_dofInfo[i].range;
    // If the current cfg lies outside the limits, push it inside.
    if(!limit.Contains(cfg[i]))
      cfg[i] = limit.ClearancePoint(cfg[i]);
  }

  // Configure on the valid Cfg.
  Configure(cfg);
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
    const size_t index = AddBody(Body(this));
    GetBody(index)->Read(_is, _cbs);
    SetBaseBody(index);

    FindMultiBodyInfo();
    return;
  }

  size_t bodyCount = ReadField<size_t>(_is, _cbs, "Failed reading body count.");

  m_baseIndex = -1;
  for(size_t i = 0; i < bodyCount && _is; ++i) {
    const size_t index = AddBody(Body(this, i));
    auto free = GetBody(index);
    free->Read(_is, _cbs);

    if(free->IsBase() && m_baseIndex == size_t(-1))
      m_baseIndex = index;
  }
  SetBaseBody(m_baseIndex);

  if(m_baseIndex == size_t(-1))
    throw ParseException(_cbs.Where(), "Active body has no base.");

  //get connection info
  std::string connectionTag = ReadFieldString(_is, _cbs,
      "Failed reading connections tag.");

  //check that there is any additional adjacency connections
    if(connectionTag == "ADJACENCIES") {
      size_t adjacencyCount = ReadField<size_t>(_is, _cbs,
          "Failed reading number of closing loop adjacency connections.");
      for(size_t m = 0; m < adjacencyCount && _is; ++m) {
        //read body indices
        int firstI = ReadField<int>(_is, _cbs, "Failed reading first closing loop index");
        int secondI = ReadField<int>(_is, _cbs, "Failed reading second closing loop index");
        m_joints.emplace_back(new Connection(this));
        m_joints.back()->SetAdjacentBodies(this, firstI, secondI);
      }
      connectionTag = ReadFieldString(_is, _cbs, "Failed reading connections tag.");
    }

  if(connectionTag != "CONNECTIONS")
    throw ParseException(_cbs.Where(),
        "Unknwon connections tag '" + connectionTag + "'."
        " Should read 'Connections'.");
  size_t connectionCount = ReadField<size_t>(_is, _cbs,
      "Failed reading number of connections.");

  for(size_t i = 0; i < connectionCount; ++i) {
    // add connection info to multibody connection map
    m_joints.emplace_back(new Connection(this));
    m_joints.back()->Read(_is, _cbs);
    m_joints.back()->SetBodies();
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
      _os << body << std::endl;
    return;
  }

  // Write free body count and free bodies.
  _os << m_bodies.size() << std::endl;
  for(const auto& body : m_bodies)
    _os << body << std::endl;

  // Write connections.
  size_t numConnection = 0;
  for(const auto& body : m_bodies)
    numConnection += body.ForwardConnectionCount();

  _os << "Connections\n"
      << numConnection
      << std::endl;

  for(const auto& body : m_bodies)
    for(size_t j = 0; j < body.ForwardConnectionCount(); ++j)
      _os << body.GetForwardConnection(j);
}

/*---------------------------------- Helpers ---------------------------------*/

void
MultiBody::
SortJoints() {
  const static auto sorter =
      [](const std::unique_ptr<Connection>& _a,
         const std::unique_ptr<Connection>& _b)
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
    if(!body.IsBase())
      body.MarkDirty();

  /// @TODO I think we should be able to remove this forced recomputation and
  ///       let it resolve lazily, need to test though.
  for(auto& body : m_bodies)
    if(body.ForwardConnectionCount() == 0)  // tree tips: leaves.
      body.GetWorldTransformation();
}


void
MultiBody::
FindMultiBodyInfo() {
  // Find COM
  m_com(0, 0, 0);
  for(auto& body : m_bodies)
    m_com += body.GetWorldPolyhedron().GetCentroid();
  m_com /= m_bodies.size();

  //Find Bounding box
  double minX, minY, minZ, maxX, maxY, maxZ;
  minX = minY = minZ = std::numeric_limits<double>::max();
  maxX = maxY = maxZ = std::numeric_limits<double>::lowest();

  for(auto& body : m_bodies) {
    const auto bbx = body.GetWorldBoundingBox();
    const auto& minVertex = bbx.m_vertexList[0];
    const auto& maxVertex = bbx.m_vertexList[7];

    minX = std::min(minX, minVertex[0]);
    maxX = std::max(maxX, maxVertex[0]);
    minY = std::min(minY, minVertex[1]);
    maxY = std::max(maxY, maxVertex[1]);
    minZ = std::min(minZ, minVertex[2]);
    maxZ = std::max(maxZ, maxVertex[2]);
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
  m_radius = m_bodies[0].GetPolyhedron().m_maxRadius;
  for(size_t i = 1; i < m_bodies.size(); ++i)
    m_radius += m_bodies[i].GetPolyhedron().m_maxRadius * 2.0;
}

/*----------------------------------------------------------------------------*/
