#include "FreeBody.h"
#include "Utilities/XMLNode.h"


FreeBody::
FreeBody(MultiBody* _owner, size_t _index)
  : Body(_owner), m_index(_index), m_bodyType(BodyType::Planar),
    m_movementType(MovementType::Translational) {
  /// @TODO Parse the label properly after we set up the robot xml file.
  m_label = std::to_string(m_index);
}


FreeBody::
FreeBody(MultiBody* _owner, XMLNode& _node, size_t _index)
  : Body(_owner, _node), m_index(_index), m_bodyType(BodyType::Planar),
    m_movementType(MovementType::Translational) {
  /// @TODO Parse the label properly after we set up the robot xml file.
  m_label = std::to_string(m_index);

  // Read geometry filename.
  m_filename = _node.Read("filename", true, "", "File containing the geometry"
      " information for this body.");

  // Call base class Read() function to setup com adjust. The COM adjust was
  // initially setup in the base class' (Body.h) constructor so we are
  // safe to call this here.
  Read(m_comAdjust);

  // Read body type.
  const string type = _node.Read("type", true, "",
      "Type of the body (volumetric, planar, fixed, or joint.");

  // Read movement type.
  const string movement = _node.Read("movement", false, "",
      "Type of the movement (rotational, or translational).");

  // Assign movement type.
  if(movement == "rotational")
    m_movementType = MovementType::Rotational;
  else if(movement == "translational")
    m_movementType = MovementType::Translational;

  // Assign body type.
  if(type == "planar") {
    m_bodyType = BodyType::Planar;

    // Movement type is required for planar.
    if(movement.empty())
      throw ParseException(_node.Where(), "Movement type is required for planar"
          " bodies. The movement type can be rotational or translational.");
  }
  else if(type == "volumetric") {
    m_bodyType = BodyType::Volumetric;

    // Movement type is required for volumetric.
    if(movement.empty())
      throw ParseException(_node.Where(), "Movement type is required for"
          " volumetric bodies. The movement type can be rotational or"
          " translational.");
  }
  else if(type == "fixed") {
    m_bodyType = BodyType::Fixed;

    const std::string transform = _node.Read("transform", true, "",
        "The transform information of this body.");

    istringstream iss(transform);
    iss >> m_transform;
  }
  else if(type == "joint")
    m_bodyType = BodyType::Joint;
  else
    throw ParseException(_node.Where(),
        "Unknown base type '" + type + "'."
        " Options are: 'planar', 'volumetric', 'fixed', or 'joint'.");

}


FreeBody::BodyType
FreeBody::
GetBodyTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "PLANAR")
    return BodyType::Planar;
  else if(_tag == "VOLUMETRIC")
    return BodyType::Volumetric;
  else if(_tag == "FIXED")
    return BodyType::Fixed;
  else if(_tag == "JOINT")
    return BodyType::Joint;
  else
    throw ParseException(_where,
        "Unknown base type '" + _tag + "'."
        " Options are: 'planar', 'volumetric', 'fixed', or 'joint'.");
}


FreeBody::MovementType
FreeBody::
GetMovementTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "ROTATIONAL")
    return MovementType::Rotational;
  else if (_tag == "TRANSLATIONAL")
    return MovementType::Translational;
  else
    throw ParseException(_where,
        "Unknown movement type '" + _tag + "'."
        " Options are: 'rotational' or 'translational'.");
}


string
FreeBody::
GetTagFromBodyType(BodyType _b) {
  switch(_b) {
    case BodyType::Planar:
      return "Planar";
    case BodyType::Volumetric:
      return "Volumetric";
    case BodyType::Fixed:
      return "Fixed";
    case BodyType::Joint:
      return "Joint";
    default:
      return "Unknown Base Type";
  }
}


string
FreeBody::
GetTagFromMovementType(MovementType _bm) {
  switch(_bm){
    case MovementType::Rotational:
      return "Rotational";
    case MovementType::Translational:
      return "Translational";
    default:
      return "Unknown Base Movement";
  }
}


Connection&
FreeBody::
GetForwardConnection(size_t _index) {
  if(_index < m_forwardConnections.size())
    return *m_forwardConnections[_index];
  else
    throw RunTimeException(WHERE,
        "Cannot access forward connection '" + ::to_string(_index) + "'.");
}


Connection&
FreeBody::
GetBackwardConnection(size_t _index) {
  if(_index < m_backwardConnections.size())
    return *m_backwardConnections[_index];
  else
    throw RunTimeException(WHERE,
        "Cannot access backward connection '" + ::to_string(_index) + "'.");
}


bool
FreeBody::
IsAdjacent(const FreeBody* const _otherBody) const {
  for(const auto& c : m_forwardConnections)
    if(c->GetNextBody() == _otherBody)
      return true;
  for(const auto& c : m_backwardConnections)
    if(c->GetPreviousBody() == _otherBody)
      return true;
  return this == _otherBody;
}


bool
FreeBody::
IsWithinI(const FreeBody* const _otherBody, size_t _i) const {
  return IsWithinI(this, _otherBody, _i, NULL);
}


void
FreeBody::
Link(Connection* _c) {
  m_forwardConnections.push_back(_c);
  _c->GetNextBody()->m_backwardConnections.push_back(_c);
  MarkDirty();
}


const Transformation&
FreeBody::
GetWorldTransformation() const {
  if(m_transformCached)
    return m_transform;
  set<size_t> visited;
  return ComputeWorldTransformation(visited);
}


Transformation&
FreeBody::
GetRenderTransformation() {
  set<size_t> visited;
  return ComputeRenderTransformation(visited);
}


void
FreeBody::
Configure(const Transformation& _transformation) {
  m_transform = _transformation;
  MarkDirty();
}


void
FreeBody::
ConfigureRender(const Transformation& _transformation) {
  m_renderTransformation = _transformation;
}


void
FreeBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  ReadOptions(_is, _cbs);

  Read(m_comAdjust);

  // Read for Base Type. If Planar or Volumetric, read in two more strings
  // If Joint skip this stuff. If Fixed read in positions like an obstacle
  string bodyTag = ReadFieldString(_is, _cbs, "Failed reading base tag."
      " Options are: planar, volumetric, fixed, or joint.");
  m_bodyType = GetBodyTypeFromTag(bodyTag, _cbs.Where());

  switch(m_bodyType) {
    case BodyType::Volumetric:
    case BodyType::Planar:
      // If base is volumetric or planar, we should parse the rotational type.
      {
        string baseMovementTag = ReadFieldString(_is, _cbs,
            "Failed reading rotation tag."
            " Options are: rotational or translational.");
        m_movementType = GetMovementTypeFromTag(baseMovementTag, _cbs.Where());
        break;
      }
    case BodyType::Fixed:
      // If base if fixed, we should read a transformation.
      {
        m_transform = ReadField<Transformation>(_is, _cbs,
            "Failed reading fixed based transformation.");
        ConfigureRender(m_transform);
        break;
      }
    case BodyType::Joint:
      // No additional parsing needed if base is a joint.
      break;
  }
}


ostream&
operator<<(ostream& _os, FreeBody& _fb){
  _os << _fb.m_filename << " ";

  _os << FreeBody::GetTagFromBodyType(_fb.m_bodyType) << " ";

  switch(_fb.m_bodyType) {
    case FreeBody::BodyType::Planar:
    case FreeBody::BodyType::Volumetric:
      _os << FreeBody::GetTagFromMovementType(_fb.m_movementType);
      break;
    case FreeBody::BodyType::Fixed:
      _os << _fb.m_transform;
    case FreeBody::BodyType::Joint:
      break;
  }

  return _os;
}


bool
FreeBody::
IsWithinI(const FreeBody* const _body1, const FreeBody* const _body2, size_t _i,
    const FreeBody* const _prevBody) const {
  if(_body1 == _body2)
    return true;

  if(_i == 0)
    return false;

  for(const auto& c : m_forwardConnections) {
    FreeBody* next = c->GetNextBody();
    if(next != _prevBody && IsWithinI(next, _body2, _i - 1, _body1))
      return true;
  }
  for(const auto& c : m_backwardConnections) {
    FreeBody* prev = c->GetPreviousBody();
    if(prev != _prevBody && IsWithinI(prev, _body2, _i - 1, _body1))
      return true;
  }
  return false;
}


const Transformation&
FreeBody::
ComputeWorldTransformation(set<size_t>& _visited) const {
  /// @note This method is marked const even though it might change the world
  ///       transformation because it is essentially completing a lazy
  ///       computation of the transform. The computation begins when we first
  ///       position the robot (thereby changing the body transforms) and lazily
  ///       resolves when we actually request the updated transform.

  // If this link has already been visited, no need to do anything.
  if(_visited.find(m_index) != _visited.end())
    return m_transform;

  // The transform is already correct if there are no backward connections
  // (this is a base link) or if we have already computed it. Otherwise, compute
  // the transform of this link from its backward connection.
  if(!m_backwardConnections.empty() and !m_transformCached) {
    const Connection& back = *m_backwardConnections[0];
    auto& transform = const_cast<Transformation&>(m_transform);
    transform =
        back.GetPreviousBody()->ComputeWorldTransformation(_visited) *
        back.GetTransformationToDHFrame() *
        back.GetDHParameters().GetTransformation() *
        back.GetTransformationToBody2();
  }

  // Mark this link as visited.
  _visited.insert(m_index);
  m_transformCached = true;

  return m_transform;
}


Transformation&
FreeBody::
ComputeRenderTransformation(set<size_t>& _visited) {
  if(_visited.find(m_index) != _visited.end()) {
    return m_renderTransformation;
  }
  else {
    _visited.insert(m_index);

    if(m_backwardConnections.empty())
      return m_renderTransformation;

    Connection& back = *m_backwardConnections[0];
    m_renderTransformation =
        back.GetPreviousBody()->ComputeRenderTransformation(_visited) *
        back.GetTransformationToDHFrame() *
        back.GetDHRenderParameters().GetTransformation() *
        back.GetTransformationToBody2();

    return m_renderTransformation;
  }
}
