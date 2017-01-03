#include "Connection.h"

#include "ActiveMultiBody.h"
#include "FreeBody.h"

size_t Connection::m_globalCounter = 0;


/*------------------------------ Construction --------------------------------*/

Connection::
Connection(MultiBody* _owner) : m_multibody(_owner),
    m_jointType(JointType::NonActuated) {
  m_globalIndex = m_globalCounter++;
}

/*----------------------------------- I/O ------------------------------------*/

Connection::JointType
Connection::
GetJointTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "REVOLUTE")
    return JointType::Revolute;
  else if(_tag == "SPHERICAL")
    return JointType::Spherical;
  else if(_tag == "NONACTUATED")
    return JointType::NonActuated;
  else
    throw ParseException(_where,
        "Unknown joint type '" + _tag + "'."
        " Options are: 'revolute', 'spherical', or 'nonactuated'.");
}


string
Connection::
GetTagFromJointType(JointType _j) {
  switch(_j) {
    case JointType::Revolute:
      return "Revolute";
    case JointType::Spherical:
      return "Spherical";
    default:
      return "Unknown Joint Type";
  }
}


void
Connection::
Read(istream& _is, CountingStreamBuffer& _cbs) {
  //body indices
  m_bodyIndices.first = ReadField<int>(_is, _cbs,
      "Failed reading previous body index.");
  m_bodyIndices.second = ReadField<int>(_is, _cbs,
      "Failed reading next body index.");

  //grab the shared_ptr to bodies
  m_bodies[0] = static_cast<ActiveMultiBody*>(m_multibody)->GetFreeBody(m_bodyIndices.first);
  m_bodies[1] = static_cast<ActiveMultiBody*>(m_multibody)->GetFreeBody(m_bodyIndices.second);

  //grab the joint type
  string connectionTypeTag = ReadFieldString(_is, _cbs,
      "Failed reading connection type."
      " Options are: revolute, spherical, or nonactuated.");
  m_jointType = GetJointTypeFromTag(connectionTypeTag, _cbs.Where());

  //grab the joint limits for revolute and spherical joints
  if(m_jointType == JointType::Revolute ||
      m_jointType == JointType::Spherical) {
    m_jointLimits[0].first = m_jointLimits[1].first = -1;
    m_jointLimits[0].second = m_jointLimits[1].second = 1;
    size_t numRange = m_jointType == JointType::Revolute ? 1 : 2;
    for(size_t i = 0; i < numRange; i++){
      string tok;
      if(_is >> tok){
        size_t del = tok.find(":");
        if(del == string::npos)
          throw ParseException(_cbs.Where(),
              "Failed reading joint range. Should be delimited by ':'.");

        istringstream minv(tok.substr(0, del)),
                      maxv(tok.substr(del+1, tok.length()));
        if(!(minv >> m_jointLimits[i].first &&
              maxv >> m_jointLimits[i].second))
          throw ParseException(_cbs.Where(), "Failed reading joint range.");
      }
      else if(numRange == 2 && i==1) //error. only 1 token provided.
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

  //make the connection
  m_bodies[0]->Link(this);
}


ostream&
operator<<(ostream& _os, const Connection& _c) {
  _os << _c.m_bodyIndices.first << " " << _c.m_bodyIndices.second << " "
      << Connection::GetTagFromJointType(_c.m_jointType);
  if(_c.m_jointType == Connection::JointType::Revolute ||
      _c.m_jointType == Connection::JointType::Spherical)
    _os << " " << _c.GetJointLimits(0).first << ":"
        << _c.GetJointLimits(0).second;
  _os << endl
      << _c.m_transformationToDHFrame << endl
      << _c.m_dhParameters << endl
      << _c.m_transformationToBody2 << endl;
  return _os;
}

/*-------------------------- Joint Information -------------------------------*/

size_t
Connection::
GetGlobalIndex() const {
  return m_globalIndex;
}


Connection::JointType
Connection::
GetConnectionType() const {
  return m_jointType;
}


const pair<double, double>&
Connection::
GetJointLimits(size_t _i) const {
  return m_jointLimits[_i];
}

/*----------------------- FreeBody Information -------------------------------*/

const FreeBody*
Connection::
GetPreviousBody() const {
  return m_bodies[0];
}


FreeBody*
Connection::
GetPreviousBody() {
  return m_bodies[0];
}


size_t
Connection::
GetPreviousBodyIndex() const {
  return m_bodyIndices.first;
}


const FreeBody*
Connection::
GetNextBody() const {
  return m_bodies[1];
}


FreeBody*
Connection::
GetNextBody() {
  return m_bodies[1];
}


size_t
Connection::
GetNextBodyIndex() const {
  return m_bodyIndices.second;
}

/*-------------------------- Transformation Info -----------------------------*/

DHParameters&
Connection::
GetDHParameters() {
  return m_dhParameters;
}


const DHParameters&
Connection::
GetDHParameters() const {
  return m_dhParameters;
}


DHParameters&
Connection::
GetDHRenderParameters() {
  return m_dhRenderParameters;
}


Transformation&
Connection::
GetTransformationToBody2() {
  return m_transformationToBody2;
}


const Transformation&
Connection::
GetTransformationToBody2() const {
  return m_transformationToBody2;
}


Transformation&
Connection::
GetTransformationToDHFrame() {
  return m_transformationToDHFrame;
}


const Transformation&
Connection::
GetTransformationToDHFrame() const {
  return m_transformationToDHFrame;
}

/*----------------------------------------------------------------------------*/
