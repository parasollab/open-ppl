#include "Connection.h"

#include "MultiBody.h"

size_t Connection::m_globalCounter = 0;

Connection::
Connection(MultiBody* _owner) : m_multibody(_owner), m_jointType(NONACTUATED) {
  m_globalIndex = m_globalCounter++;
}

Connection::
Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2,
    const Transformation& _transformationToBody2,
    const DHparameters& _dhparameters,
    const Transformation& _transformationToDHFrame) :
  m_multibody(NULL),
  m_transformationToBody2(_transformationToBody2),
  m_transformationToDHFrame(_transformationToDHFrame),
  m_dhParameters(_dhparameters),
  m_jointType(NONACTUATED) {
    m_globalIndex = m_globalCounter++;
    m_bodies[0] = _body1;
    m_bodies[1] = _body2;
  }

Connection::JointType
Connection::
GetJointTypeFromTag(const string& _tag, const string& _where) {
  if(_tag == "REVOLUTE")
    return Connection::REVOLUTE;
  else if (_tag == "SPHERICAL")
    return Connection::SPHERICAL;
  else if(_tag == "NONACTUATED")
    return Connection::NONACTUATED;
  else
    throw ParseException(_where,
        "Unknown joint type '" + _tag + "'."
        " Options are: 'revolute', 'spherical', or 'nonactuated'.");
}

string
Connection::
GetTagFromJointType(const Connection::JointType& _jt){
  switch(_jt){
    case REVOLUTE:
      return "REVOLUTE";
    case SPHERICAL:
      return "SPHERICAL";
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
  m_bodies[0] = m_multibody->GetFreeBody(m_bodyIndices.first);
  m_bodies[1] = m_multibody->GetFreeBody(m_bodyIndices.second);

  //grab the joint type
  string connectionTypeTag = ReadFieldString(_is, _cbs,
      "Failed reading connection type."
      " Options are: revolute, spherical, or nonactuated.");
  m_jointType = GetJointTypeFromTag(connectionTypeTag, _cbs.Where());

  //grab the joint limits for revolute and spherical joints
  if(m_jointType == Connection::REVOLUTE ||
      m_jointType == Connection::SPHERICAL) {
    m_jointLimits[0].first = m_jointLimits[1].first = -1;
    m_jointLimits[0].second = m_jointLimits[1].second = 1;
    size_t numRange = m_jointType == Connection::REVOLUTE ? 1 : 2;
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
  m_dhParameters = ReadField<DHparameters>(_is, _cbs,
      "Failed reading DH parameters.");

  //transformation to next body
  m_transformationToBody2 = ReadField<Transformation>(_is, _cbs,
      "Failed reading transformation to next body.");

  //make the connection
  m_bodies[0]->Link(*this);
}

ostream&
operator<<(ostream& _os, const Connection& _c) {
  return _os << _c.m_bodyIndices.first << " " << _c.m_bodyIndices.second << " "
    << Connection::GetTagFromJointType(_c.m_jointType) << endl
    << _c.m_transformationToDHFrame << " " << _c.m_dhParameters << " "
    << _c.m_transformationToBody2;
}

