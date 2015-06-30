#include "FreeBody.h"

#include "ActiveMultiBody.h"

FreeBody::
FreeBody(MultiBody* _owner) : Body(_owner),
  m_isBase(false),
  m_bodyType(BodyType::Planar),
  m_movementType(MovementType::Translational) {
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
  if (_index < m_forwardConnection.size())
    return m_forwardConnection[_index];
  else{
    cerr << "Error, in FreeBody::GetForwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

Connection&
FreeBody::
GetBackwardConnection(size_t _index) {
  if (_index < m_backwardConnection.size())
    return m_backwardConnection[_index];
  else{
    cerr << "Error, in FreeBody::GetBackwardConnection: requesting connection outside of bounds\n\n";
    exit(-1);
  }
}

bool
FreeBody::IsAdjacent(shared_ptr<FreeBody> _otherBody) {
  for(vector<Connection>::iterator C = m_forwardConnection.begin(); C != m_forwardConnection.end(); ++C)
    if(C->GetNextBody() == _otherBody)
      return true;
  for(vector<Connection>::iterator C = m_backwardConnection.begin(); C != m_backwardConnection.end(); ++C)
    if(C->GetPreviousBody() == _otherBody)
      return true;
  return this == _otherBody.get();
}

bool
FreeBody::IsWithinI(shared_ptr<FreeBody> _otherBody, int _i){
  return IsWithinIHelper(this, _otherBody.get(), _i, NULL);
}

bool
FreeBody::IsWithinIHelper(FreeBody* _body1, FreeBody* _body2, int _i, FreeBody* _prevBody){
  if(_body1 == _body2)
    return true;

  if(_i == 0)
    return false;

  typedef vector<Connection>::iterator CIT;
  for(CIT C = _body1->m_forwardConnection.begin(); C != _body1->m_forwardConnection.end(); ++C) {
    FreeBody* next = C->GetNextBody().get();
    if(next != _prevBody && IsWithinIHelper(next, _body2, _i-1, _body1))
      return true;
  }
  for(CIT C =_body1->m_backwardConnection.begin(); C != _body1->m_backwardConnection.end(); ++C) {
    FreeBody* prev = C->GetPreviousBody().get();
    if(prev != _prevBody && IsWithinIHelper(prev, _body2, _i-1, _body1))
      return true;
  }
  return false;
}

void
FreeBody::
Link(const shared_ptr<FreeBody>& _otherBody,
    const Transformation& _transformationToBody2,
    const DHparameters& _dhparameters,
    const Transformation& _transformationToDHFrame) {
  Connection c(shared_ptr<FreeBody>(this), _otherBody, _transformationToBody2,
      _dhparameters, _transformationToDHFrame);
  Link(c);
}

void
FreeBody::
Link(const Connection& _c) {
  AddForwardConnection(_c);
  _c.GetNextBody()->AddBackwardConnection(_c);
  m_worldPolyhedronAvailable=false;
  m_centerOfMassAvailable=false;
}

Transformation&
FreeBody::
GetWorldTransformation() {
  set<int> visited;
  return this->ComputeWorldTransformation(visited);
}

void
FreeBody::
Configure(Transformation& _transformation) {
  m_worldTransformation = _transformation;
  m_centerOfMassAvailable = false;
  m_worldPolyhedronAvailable = false;
}

void
FreeBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  Read();

  //Read for Base Type.  If Planar or Volumetric, read in two more strings
  //If Joint skip this stuff. If Fixed read in positions like an obstacle
  string bodyTag = ReadFieldString(_is, _cbs, "Failed reading base tag."
      " Options are: planar, volumetric, fixed, or joint.");
  m_bodyType = GetBodyTypeFromTag(bodyTag, _cbs.Where());

  switch(m_bodyType) {
    //if base is volumetric or planar we should parse the rotational type
    case BodyType::Volumetric:
    case BodyType::Planar:
      {
        m_isBase = true;
        string baseMovementTag = ReadFieldString(_is, _cbs,
            "Failed reading rotation tag."
            " Options are: rotational or translational.");
        m_movementType =
          GetMovementTypeFromTag(baseMovementTag, _cbs.Where());
        break;
      }

      //if base if fixed we should read a transformation
    case BodyType::Fixed:
      {
        m_isBase = true;
        m_worldTransformation =
          ReadField<Transformation>(_is, _cbs,
              "Failed reading fixed based transformation.");
        break;
      }

      //if the base is a joint nothing additional is parsed
    case BodyType::Joint:
      break;
  }
}

Transformation&
FreeBody::
ComputeWorldTransformation(set<int>& _visited) {
  m_centerOfMassAvailable = false;
  m_worldPolyhedronAvailable = false;

  ActiveMultiBody* multibody = dynamic_cast<ActiveMultiBody*>(m_multibody);
  if(_visited.find(multibody->GetFreeBodyIndex(*this)) != _visited.end()) {
    return m_worldTransformation;
  }
  else {
    _visited.insert(multibody->GetFreeBodyIndex(*this));

    //for the case when the base is a freebody.
    if(m_backwardConnection.empty())
      return m_worldTransformation;

    Connection& back = m_backwardConnection[0];
    Transformation dh =
      back.GetDHparameters().GetTransformation();
    m_worldTransformation =
      ((FreeBody*)(back.GetPreviousBody().get()))->
      ComputeWorldTransformation(_visited)
      * back.GetTransformationToDHFrame()
      * dh
      * back.GetTransformationToBody2();

    return m_worldTransformation;
  }
  //return m_worldTransformation;
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
      _os << _fb.m_worldTransformation;
    case FreeBody::BodyType::Joint:
      break;
  }

  return _os;
}

