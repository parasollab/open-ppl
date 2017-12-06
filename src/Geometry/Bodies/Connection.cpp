#include "Connection.h"

#include "Body.h"
#include "MultiBody.h"
#include "Utilities/XMLNode.h"

#include <algorithm>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Local Functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/// Parse the connection type from a string tag.
/// @param _tag The string tag to parse.
/// @param _where Location of \p _tag for error reporting.
/// @return The joint type represented by _tag.
Connection::JointType
GetJointTypeFromTag(std::string _tag, const std::string& _where) {
  std::transform(_tag.begin(), _tag.end(), _tag.begin(), ::tolower);
  if(_tag == "revolute")
    return Connection::JointType::Revolute;
  else if(_tag == "spherical")
    return Connection::JointType::Spherical;
  else if(_tag == "nonactuated")
    return Connection::JointType::NonActuated;
  else
    throw ParseException(_where,
        "Unknown joint type '" + _tag + "'."
        " Options are: 'revolute', 'spherical', or 'nonactuated'.");
}


/// Create a string tag for a given connection type.
/// @param _j Joint type
/// @return String representation of _j.
std::string
GetTagFromJointType(const Connection::JointType _j) {
  switch(_j) {
    case Connection::JointType::Revolute:
      return "Revolute";
    case Connection::JointType::Spherical:
      return "Spherical";
    default:
      return "Unknown Joint Type";
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Connection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

Connection::
Connection(MultiBody* const _owner)
  : m_multibody(_owner),
    m_bodies{nullptr, nullptr},
    m_jointType(JointType::NonActuated)
{ }


Connection::
Connection(MultiBody* const _owner, XMLNode& _node)
  : Connection(_owner) {
  // Get the indices of the two bodies that are connected.
  const size_t parentIndex = _node.Read("parentIndex", true, size_t(0), size_t(0),
      std::numeric_limits<size_t>::max(), "Index of the parent body.");
  const size_t childIndex = _node.Read("childIndex", true, size_t(0), size_t(0),
      std::numeric_limits<size_t>::max(), "Index of the child body.");
  SetBodies(m_multibody, parentIndex, childIndex);

  // Read the joint info.

  // Read joint type.
  std::string joint = _node.Read("joint", true, "", "Type of joint.");
  std::transform(joint.begin(), joint.end(), joint.begin(), ::toupper);
  m_jointType = GetJointTypeFromTag(joint, _node.Where());

  // Read the first joint range.
  {
    const std::string rangeString = _node.Read("range", true, "",
        "Range of the joint.");
    std::istringstream buffer(rangeString);
    buffer >> m_jointRange[0];
  }

  // Spherical joints have two joint ranges, read the second now.
  if(IsSpherical()) {
    const std::string rangeString2 = _node.Read("range2", true, "",
        "Range of the joint about the second axis.");
    std::istringstream buffer(rangeString2);
    buffer >> m_jointRange[1];
  }

  // Read the transformations and DH params.

  // Transform from parent to DH frame.
  {
    const std::string parentToDHString = _node.Read("transformParentToDH", true,
        "", "Transformation parameters of parent to DH frame.");
    std::istringstream buffer(parentToDHString);
    while(buffer)
      buffer >> m_transformationToDHFrame;
  }

  // DH params.
  {
    const std::string dhParamsString = _node.Read("initialDHParams", true, "",
        "DH parameters.");
    std::istringstream buffer(dhParamsString);
    while(buffer)
      buffer >> m_dhParameters;
  }

  // Transform from DH to com.
  {
    const std::string dhToChildString = _node.Read("transformDHToChild", true,
        "", "Transformation paremeters of DH to com.");
    std::istringstream buffer(dhToChildString);
    while(buffer)
      buffer >> m_transformationToBody2;
  }
}


Connection::
Connection(const Connection& _other)
  : m_multibody(nullptr),
    m_bodies{nullptr, nullptr},
    m_transformationToBody2(_other.m_transformationToBody2),
    m_transformationToDHFrame(_other.m_transformationToDHFrame),
    m_dhParameters(_other.m_dhParameters),
    m_dhRenderParameters(_other.m_dhRenderParameters),
    m_jointType(_other.m_jointType),
    m_bodyIndices(_other.m_bodyIndices),
    m_jointRange(_other.m_jointRange)
{ }

/*-------------------------------- Assignment --------------------------------*/

Connection&
Connection::
operator=(const Connection& _other) {
  m_multibody = nullptr;
  m_bodies = {nullptr, nullptr};
  m_transformationToBody2 = _other.m_transformationToBody2;
  m_transformationToDHFrame = _other.m_transformationToDHFrame;
  m_dhParameters = _other.m_dhParameters;
  m_dhRenderParameters = _other.m_dhRenderParameters;
  m_jointType = _other.m_jointType;
  m_bodyIndices = _other.m_bodyIndices;
  m_jointRange = _other.m_jointRange;

  return *this;
}

/*----------------------------------- I/O ------------------------------------*/

void
Connection::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  //body indices
  const size_t parentIndex = ReadField<int>(_is, _cbs,
      "Failed reading parent body index.");
  const size_t childIndex = ReadField<int>(_is, _cbs,
      "Failed reading child body index.");
  SetBodies(m_multibody, parentIndex, childIndex);

  //grab the joint type
  std::string connectionTypeTag = ReadFieldString(_is, _cbs,
      "Failed reading connection type."
      " Options are: revolute, spherical, or nonactuated.");
  m_jointType = GetJointTypeFromTag(connectionTypeTag, _cbs.Where());

  //grab the joint limits for revolute and spherical joints
  if(IsRevolute() || IsSpherical()) {
    m_jointRange[0].min = m_jointRange[1].min = -1;
    m_jointRange[0].max = m_jointRange[1].max = 1;
    size_t numRange = IsRevolute() ? 1 : 2;
    for(size_t i = 0; i < numRange; ++i) {
      std::string tok;
      if(_is >> tok) {
        size_t del = tok.find(":");
        if(del == std::string::npos)
          throw ParseException(_cbs.Where(),
              "Failed reading joint range. Should be delimited by ':'.");

        std::istringstream minv(tok.substr(0, del)),
                      maxv(tok.substr(del+1, tok.length()));
        if(!(minv >> m_jointRange[i].min &&
              maxv >> m_jointRange[i].max))
          throw ParseException(_cbs.Where(), "Failed reading joint range.");
      }
      else if(numRange == 2 && i == 1) //error. only 1 token provided.
        throw ParseException(_cbs.Where(),
            "Failed reading joint ranges. Only one provided.");
    }
  }

  //transformation to DHFrame
  m_transformationToDHFrame = ReadField<Transformation>(_is, _cbs,
      "Failed reading transformation to DH frame.");

  //DH parameters
  m_dhParameters = ReadField<DHParameters>(_is, _cbs,
      "Failed reading DH parameters.");

  //transformation to next body
  m_transformationToBody2 = ReadField<Transformation>(_is, _cbs,
      "Failed reading transformation to next body.");
}


void
Connection::
SetBodies(MultiBody* const _owner, const size_t _parentIndex,
    const size_t _childIndex) {
  // If the bodies are already set, this is an error. Once they are set, the
  // multibody will be affected if we start changing things.
  const bool alreadySet = m_bodies[0] != nullptr or m_bodies[1] != nullptr;
  if(alreadySet)
    throw RunTimeException(WHERE, "Cannot change the connected bodies once they "
        "are set.");

  // Set the parent and child indexes.
  m_bodyIndices.first = _parentIndex;
  m_bodyIndices.second = _childIndex;

  // Set the owning multibody.
  m_multibody = _owner;

  // Retrieve the body pointers.
  m_bodies[0] = m_multibody->GetBody(m_bodyIndices.first);
  m_bodies[1] = m_multibody->GetBody(m_bodyIndices.second);

  // Update the bodies.
  m_bodies[0]->LinkForward(this);
  m_bodies[1]->LinkBackward(this);
}


void
Connection::
SetBodies(MultiBody* const _owner) {
  SetBodies(_owner, m_bodyIndices.first, m_bodyIndices.second);
}


std::ostream&
operator<<(std::ostream& _os, const Connection& _c) {
  _os << _c.m_bodyIndices.first << " "
      << _c.m_bodyIndices.second << " "
      << GetTagFromJointType(_c.m_jointType);

  if(_c.IsRevolute() or _c.IsSpherical())
    _os << " " << _c.GetJointRange(0);
  _os << std::endl
      << _c.m_transformationToDHFrame << std::endl
      << _c.m_dhParameters << std::endl
      << _c.m_transformationToBody2 << std::endl;
  return _os;
}

/*-------------------------- Joint Information -------------------------------*/

Connection::JointType
Connection::
GetConnectionType() const noexcept {
  return m_jointType;
}


bool
Connection::
IsRevolute() const noexcept {
  return m_jointType == JointType::Revolute;
}


bool
Connection::
IsSpherical() const noexcept {
  return m_jointType == JointType::Spherical;
}


bool
Connection::
IsNonActuated() const noexcept {
  return m_jointType == JointType::NonActuated;
}


const Range<double>&
Connection::
GetJointRange(size_t _i) const noexcept {
  return m_jointRange[_i];
}

/*----------------------------- Body Information -----------------------------*/

const Body*
Connection::
GetPreviousBody() const noexcept {
  return m_bodies[0];
}


Body*
Connection::
GetPreviousBody() noexcept {
  return m_bodies[0];
}


size_t
Connection::
GetPreviousBodyIndex() const noexcept {
  return m_bodyIndices.first;
}


const Body*
Connection::
GetNextBody() const noexcept {
  return m_bodies[1];
}


Body*
Connection::
GetNextBody() noexcept {
  return m_bodies[1];
}


size_t
Connection::
GetNextBodyIndex() const noexcept {
  return m_bodyIndices.second;
}

/*-------------------------- Transformation Info -----------------------------*/

DHParameters&
Connection::
GetDHParameters() noexcept {
  return m_dhParameters;
}


const DHParameters&
Connection::
GetDHParameters() const noexcept {
  return m_dhParameters;
}


DHParameters&
Connection::
GetDHRenderParameters() noexcept {
  return m_dhRenderParameters;
}


Transformation&
Connection::
GetTransformationToBody2() noexcept {
  return m_transformationToBody2;
}


const Transformation&
Connection::
GetTransformationToBody2() const noexcept {
  return m_transformationToBody2;
}


Transformation&
Connection::
GetTransformationToDHFrame() noexcept {
  return m_transformationToDHFrame;
}


const Transformation&
Connection::
GetTransformationToDHFrame() const noexcept {
  return m_transformationToDHFrame;
}

/*----------------------------------------------------------------------------*/
