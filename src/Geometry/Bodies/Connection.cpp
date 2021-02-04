#include "Connection.h"
#include "EulerAngle.h"

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
    throw ParseException(_where) <<  "Unknown joint type '" << _tag << "'."
                                 << " Options are: 'revolute', 'spherical', or "
                                 << "'nonactuated'.";
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
    case Connection::JointType::NonActuated:
      return "Nonactuated";
    default:
      return "Unknown Joint Type";
  }
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Connection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

Connection::
Connection(MultiBody* const _owner)
  : m_multibody(_owner),
    m_jointType(JointType::NonActuated)
{ 
  m_jointParamType=JointParamType::DH;
}


Connection::
Connection(MultiBody* const _owner, XMLNode& _node)
  : Connection(_owner) {
  // Get the indices of the two bodies that are connected.
  m_bodyIndices.first = _node.Read("parentIndex", true, size_t(0), size_t(0),
      std::numeric_limits<size_t>::max(), "Index of the parent body.");
  m_bodyIndices.second = _node.Read("childIndex", true, size_t(0), size_t(0),
      std::numeric_limits<size_t>::max(), "Index of the child body.");

  // Read the joint info.

  // Read joint type.
  const std::string joint = _node.Read("joint", true, "", "Type of joint.");
  m_jointType = GetJointTypeFromTag(joint, _node.Where());

  // Read the first joint range.
  if(!IsNonActuated()) {
    {
      const std::string rangeString = _node.Read("range", true, "",
          "Range of the joint.");
      std::istringstream buffer(rangeString);
      buffer >> m_jointRange[0];
      if(!buffer)
        throw ParseException(_node.Where()) << "range is ill-formed.";
    }

    // Spherical joints have two joint ranges, read the second now.
    if(IsSpherical()) {
      const std::string rangeString2 = _node.Read("range2", true, "",
          "Range of the joint about the second axis.");
      std::istringstream buffer(rangeString2);
      buffer >> m_jointRange[1];
      if(!buffer)
        throw ParseException(_node.Where()) << "range2 is ill-formed.";
    }
  }

  // Read the transformations and DH params.

  // Transform from parent to DH frame.
  {
    const std::string parentToDHString = _node.Read("transformParentToDH", true,
        "", "Transformation parameters of parent to DH frame.");
    std::istringstream buffer(parentToDHString);
    buffer >> m_transformationToDHFrame;
    m_jointParamType=JointParamType::DH;
    if(!buffer)
      throw ParseException(_node.Where()) << "transformParentToDH is "
                                          << "ill-formed.";
  }

  // DH params.
  {
    const std::string dhParamsString = _node.Read("initialDHParams", true, "",
        "DH parameters.");
    std::istringstream buffer(dhParamsString);
    buffer >> m_dhParameters;
    if(!buffer)
      throw ParseException(_node.Where()) << "initialDHParams is ill-formed.";
  }

  // Transform from DH to com.
  {
    const std::string dhToChildString = _node.Read("transformDHToChild", true,
        "", "Transformation paremeters of DH to com.");
    std::istringstream buffer(dhToChildString);
    buffer >> m_transformationToBody2;
    if(!buffer)
      throw ParseException(_node.Where()) << "transformDHToChild is "
                                          << "ill-formed.";
  }
}


Connection::
Connection(const Connection& _other) {
  *this = _other;
}


Connection::
Connection(Connection&&) = default;

/*-------------------------------- Assignment --------------------------------*/

Connection&
Connection::
operator=(const Connection& _other) {
  if(this != &_other) {
    m_multibody                  = nullptr;
    m_transformationToBody2      = _other.m_transformationToBody2;
    m_transformationToDHFrame    = _other.m_transformationToDHFrame;
    m_dhParameters               = _other.m_dhParameters;
    m_jointType                  = _other.m_jointType;
    m_bodyIndices                = _other.m_bodyIndices;
    m_jointRange                 = _other.m_jointRange;
    m_jointParamType             = _other.m_jointParamType;
    m_transformationToChildFrame = _other.m_transformationToChildFrame;
    m_jointAxis                  = _other.m_jointAxis;
  }

  return *this;
}


Connection&
Connection::
operator=(Connection&&) = default;

/*----------------------------------- I/O ------------------------------------*/

void
Connection::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  //body indices
  m_bodyIndices.first = ReadField<size_t>(_is, _cbs,
      "Failed reading parent body index.");
  m_bodyIndices.second = ReadField<size_t>(_is, _cbs,
      "Failed reading child body index.");

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

  m_jointParamType = JointParamType::DH;

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
TranslateURDFJoint(const std::shared_ptr<urdf::Joint>& _joint,
                   const std::pair<size_t,size_t> _bodyIndices,
                   const Vector3d _bodyPosition,
                   const MatrixOrientation _bodyOrientation) {

  // body indices
  m_bodyIndices = _bodyIndices;

  m_jointParamType=JointParamType::URDF;

  // grab the joint type
  switch(_joint->type) {
    case urdf::Joint::REVOLUTE :
      m_jointType = Connection::JointType::Revolute;
      break;
    case urdf::Joint::FIXED :
      m_jointType = Connection::JointType::NonActuated;
      break;
    default :
      throw RunTimeException(WHERE) << "Unsupported joint type." 
                                    << std::endl;
  }

  // grab the joint limits for revolute and spherical joints
  if(IsRevolute() || IsSpherical()) {
    m_jointRange[0].min = m_jointRange[1].min = -1;
    m_jointRange[0].max = m_jointRange[1].max = 1;
    size_t numRange = IsRevolute() ? 1 : 2;

    auto limits = _joint->limits;

    for(size_t i = 0; i < numRange; ++i) {
      m_jointRange[i].min = limits->lower/PI;
      m_jointRange[i].max = limits->upper/PI;
      m_jointValues.push_back(0);
    }
  }

  auto& transform = _joint->parent_to_joint_origin_transform;
  Vector3d position = {transform.position.x, 
                       transform.position.y, 
                       transform.position.z};

  Quaternion quaternion(transform.rotation.w,
                        {transform.rotation.x,
                        transform.rotation.y,
                        transform.rotation.z});

  MatrixOrientation orientation(quaternion);

  //transformation to Child Frame 
  m_transformationToChildFrame =  Transformation(position,
                                                 orientation);

  //transformation to next body
  m_transformationToBody2 = Transformation(_bodyPosition,
                                          _bodyOrientation);

  if(IsRevolute())
    m_jointAxis = {_joint->axis.x,_joint->axis.y,_joint->axis.z};

}

void
Connection::
SetBodies(MultiBody* const _owner, const size_t _parentIndex,
    const size_t _childIndex) {
  // Set the parent and child indexes.
  m_bodyIndices.first = _parentIndex;
  m_bodyIndices.second = _childIndex;

  // Set the owning multibody.
  if(_owner)
    m_multibody = _owner;

  // Update the bodies.
  GetPreviousBody()->LinkForward(this);
  GetNextBody()->LinkBackward(this);
}


void
Connection::
SetBodies(MultiBody* const _owner) {
  SetBodies(_owner, m_bodyIndices.first, m_bodyIndices.second);
}


void
Connection::
SetAdjacentBodies(MultiBody* const _owner, const size_t _firstIndex,
    const size_t _secondIndex) {

  // Set the parent and child indexes.
  m_bodyIndices.first = _firstIndex;
  m_bodyIndices.second = _secondIndex;

  // Set the owning multibody.
  m_multibody = _owner;

  // Update the bodies.
  m_multibody->GetBody(_firstIndex)->LinkAdjacency(this);
  m_multibody->GetBody(_secondIndex)->LinkAdjacency(this);
}

/*-------------------------- Joint Information -------------------------------*/

Connection::JointType
Connection::
GetConnectionType() const noexcept {
  return m_jointType;
}

Connection::JointParamType
Connection::    
GetParamType() const noexcept {
  return m_jointParamType;
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
GetJointRange(const size_t _i) const noexcept {
  return m_jointRange[_i];
}


void
Connection::
SetJointRange(const size_t _i, const Range<double>& _r) noexcept {
  m_jointRange[_i] = _r;
}

/*----------------------------- Body Information -----------------------------*/

const Body*
Connection::
GetPreviousBody() const noexcept {
  return m_multibody->GetBody(m_bodyIndices.first);
}


Body*
Connection::
GetPreviousBody() noexcept {
  return m_multibody->GetBody(m_bodyIndices.first);
}


size_t
Connection::
GetPreviousBodyIndex() const noexcept {
  return m_bodyIndices.first;
}


const Body*
Connection::
GetNextBody() const noexcept {
  return m_multibody->GetBody(m_bodyIndices.second);
}


Body*
Connection::
GetNextBody() noexcept {
  return m_multibody->GetBody(m_bodyIndices.second);
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

Transformation&
Connection::
GetTransformationToChildFrame() noexcept {
  return m_transformationToChildFrame;
}

const Transformation& 
Connection::
GetTransformationToChildFrame() const noexcept {
  return m_transformationToChildFrame;
}

Vector3d&
Connection::
GetJointAxis() noexcept {
  return m_jointAxis;
}

const Vector3d&
Connection::
GetJointAxis() const noexcept {
  return m_jointAxis;
}
    
const std::vector<double> 
Connection::
GetJointValues() {

  std::vector<double> values;

  switch(m_jointParamType) {
    case JointParamType::DH:

      values.push_back(m_dhParameters.m_theta / PI);

      if(GetConnectionType() == Connection::JointType::Spherical) {
        values.push_back(m_dhParameters.m_alpha / PI);
      }
      break;

    case JointParamType::URDF:
      values = m_jointValues;
  };

  return values;
}

void
Connection::
SetJointValues(std::vector<double> _values) {

  //TODO::Add check on the number of values

  switch(m_jointParamType) {

    case JointParamType::DH:

      m_dhParameters.m_theta = (_values[0]);
      if(GetConnectionType() == Connection::JointType::Spherical)
        m_dhParameters.m_alpha = (_values[1]);
      break;

    case JointParamType::URDF:

      m_jointValues = _values;
      break;
  };

}

const Transformation
Connection::
GetTransformationFromJoint() const noexcept {
  
  switch(m_jointParamType) {
    case JointParamType::DH:
      return GetTransformationToDHFrame() *
             m_dhParameters.GetTransformation() *
             GetTransformationToBody2();
      break;
    case JointParamType::URDF:
      return GetURDFTransformation();
      break;
  };

  return Transformation();
}


const Transformation
Connection::
GetURDFTransformation() const noexcept {

  if(GetConnectionType() == Connection::JointType::NonActuated) {
    return GetPreviousBody()->GetTransformationToURDFReferenceFrame() *
      GetTransformationToChildFrame() *
      GetTransformationToBody2();
  }

  return GetPreviousBody()->GetTransformationToURDFReferenceFrame() *
         GetTransformationToChildFrame() *
         ApplyURDFJointValues();
}

const Transformation
Connection::
ApplyURDFJointValues() const noexcept {


  if(GetConnectionType() != Connection::JointType::Revolute)
    throw RunTimeException(WHERE) << "Non revolute joints not yet supported for URDF modeling.";

  // define reused functions
  double ux,uy,uz,C,S,t;
  ux = m_jointAxis[0];
  uy = m_jointAxis[1];
  uz = m_jointAxis[2];

  C = cos(m_jointValues[0]);
  S = sin(m_jointValues[0]);
  t = 1 - C;
  
  Matrix<3,3> translation;
  //first row
  translation[0][0] = t*(ux*ux) + C;
  translation[0][1] = t*ux*uy - S*uz;
  translation[0][2] = t*ux*uz + S*uy;
  
  //second row
  translation[1][0] = t*ux*uy + S*uz;
  translation[1][1] = t*(uy*uy) + C;
  translation[1][2] = t*uy*uz - S*ux;

  //third row
  translation[2][0] = t*ux*uz - S*uy;
  translation[2][1] = t*uy*uz + S*ux;
  translation[2][2] = t*(uz*uz) + C;

  
  Transformation rotAboutAxis(Vector3d(),translation);


  const auto& t2b2 = GetTransformationToBody2();

//  return Transformation(t2b2.translation(),translation);
  return rotAboutAxis * t2b2;
}
/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Connection::JointType& _j) {
  return _os << GetTagFromJointType(_j);
}


std::ostream&
operator<<(std::ostream& _os, const Connection& _c) {
  _os << _c.GetPreviousBodyIndex() << " "
      << _c.GetNextBodyIndex() << " "
      << _c.GetConnectionType();

  if(_c.IsRevolute() or _c.IsSpherical())
    _os << " " << _c.GetJointRange(0);

  if(_c.IsSpherical())
    _os << " " << _c.GetJointRange(1);

  _os << "\n" << _c.GetTransformationToDHFrame()
      << "\n" << _c.GetDHParameters()
      << "\n" << _c.GetTransformationToBody2()
      << std::endl;

  return _os;
}

/*----------------------------------------------------------------------------*/
