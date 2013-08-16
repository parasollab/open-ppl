#include "Connection.h"
#include "MultiBody.h"

Connection::Connection(MultiBody* _owner) : m_multibody(_owner), m_jointType(NONACTUATED) {
  m_globalIndex = m_globalCounter++;
}

Connection::Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2) :
  m_multibody(NULL), m_jointType(NONACTUATED) {
  m_globalIndex = m_globalCounter++;
  m_bodies[0] = _body1;
  m_bodies[1] = _body2;
}

Connection::Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2,
    const Transformation & _transformationToBody2,
    const DHparameters & _dhparameters,
    const Transformation & _transformationToDHFrame)
  : m_multibody(NULL),
  m_transformationToBody2(_transformationToBody2),
  m_transformationToDHFrame(_transformationToDHFrame),
  m_dhParameters(_dhparameters),
  m_jointType(NONACTUATED) {
    m_globalIndex = m_globalCounter++;
    m_bodies[0] = _body1;
    m_bodies[1] = _body2;
  }

Connection::~Connection() {}

Connection::JointType
Connection::GetJointTypeFromTag(const string _tag){
  if(_tag == "REVOLUTE")
    return Connection::REVOLUTE;
  else if (_tag == "SPHERICAL")
    return Connection::SPHERICAL;
  else if(_tag == "NONACTUATED")
    return Connection::NONACTUATED;
  else {
    cerr << "Error::Incorrect Joint Type Specified::" << _tag << endl;
    cerr << "Choices are:: Revolute, Spherical or Nonactuated" << endl;
    exit(1);
  }
}

string
Connection::GetTagFromJointType(const Connection::JointType& _jt){
  switch(_jt){
    case REVOLUTE:
      return "REVOLUTE";
    case SPHERICAL:
      return "SPHERICAL";
    default:
      return "Unknown Joint Type";
  }
}

ostream&
operator<<(ostream& _os, const Connection& _c) {
  return _os << _c.m_bodyIndices.first << " " << _c.m_bodyIndices.second << " "
    << Connection::GetTagFromJointType(_c.m_jointType) << endl
    << _c.m_transformationToDHFrame << " " << _c.m_dhParameters << " "
    << _c.m_transformationToBody2;
}

istream&
operator>>(istream& _is, Connection& _c){
  //body indices
  _c.m_bodyIndices.first = ReadField<int>(_is, "Previous Body Index");
  _c.m_bodyIndices.second = ReadField<int>(_is, "Next Body Index");

  //grab the shared_ptr to bodies
  _c.m_bodies[0] = _c.m_multibody->GetFreeBody(_c.m_bodyIndices.first);
  _c.m_bodies[1] = _c.m_multibody->GetFreeBody(_c.m_bodyIndices.second);

  //grab the joint type
  string connectionTypeTag = ReadFieldString(_is, "Connection Type");
  _c.m_jointType = Connection::GetJointTypeFromTag(connectionTypeTag);

  //grab the joint limits for revolute and spherical joints
  if(_c.m_jointType == Connection::REVOLUTE || _c.m_jointType == Connection::SPHERICAL){
    _c.m_jointLimits[0].first = _c.m_jointLimits[1].first = -1;
    _c.m_jointLimits[0].second = _c.m_jointLimits[1].second = 1;
    size_t numRange = _c.m_jointType == Connection::REVOLUTE ? 1 : 2;
    for(size_t i = 0; i<numRange; i++){
      string tok;
      if(_is >> tok){
        size_t del = tok.find(":");
        if(del == string::npos){
          cerr << "Error::Reading joint range " << i << ". Should be delimited by ':'." << endl;
          exit(1);
        }
        istringstream minv(tok.substr(0,del)), maxv(tok.substr(del+1, tok.length()));
        if(!(minv>>_c.m_jointLimits[i].first && maxv>>_c.m_jointLimits[i].second)){
          cerr << "Error::Reading joint range " << i << "." << endl;
          exit(1);
        }
      }
      else if(numRange == 2 && i==1) { //error. only 1 token provided.
        cerr << "Error::Reading spherical joint ranges. Only one provided." << endl;
        exit(1);
      }
    }
  }

  //transformation to DHFrame
  _c.m_transformationToDHFrame =
    ReadField<Transformation>(_is, "Transformation to DH frame");

  //DH parameters
  _c.m_dhParameters = ReadField<DHparameters>(_is, "DH Parameters");

  //transformation to next body
  _c.m_transformationToBody2 =
    ReadField<Transformation>(_is, "Transform to next body");

  //make the connection
  _c.m_bodies[0]->Link(_c);

  return _is;
}

bool
Connection::operator==(const Connection& c) const {
  return
    m_bodies[0] == c.m_bodies[0] &&
    m_bodies[1] == c.m_bodies[1] &&
    m_transformationToBody2 == c.m_transformationToBody2 &&
    m_transformationToDHFrame == c.m_transformationToDHFrame &&
    m_dhParameters == c.m_dhParameters &&
    m_jointType == c.m_jointType;
}

size_t Connection::m_globalCounter = 0;
