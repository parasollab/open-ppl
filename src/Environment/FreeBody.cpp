#include "FreeBody.h"

FreeBody::
FreeBody(MultiBody* _owner, size_t _index) : Body(_owner),
  m_index(_index),
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

void
FreeBody::
SetClosureIndices(pair<size_t, size_t> _connection) {
  m_closureIndices.push_back(_connection);
}

bool
FreeBody::
IsAdjacent(shared_ptr<FreeBody> _otherBody) const {
  for(const auto& c : m_forwardConnections)
    if(c->GetNextBody() == _otherBody)
      return true;
  for(const auto& c : m_backwardConnections)
    if(c->GetPreviousBody() == _otherBody)
      return true;
  //Addition 1: for branched structures: check if there is a shared parent
  for(const auto& c1: m_backwardConnections) {
    Connection c2 = _otherBody->GetBackwardConnection(0);
    if(c1->GetPreviousBody() == c2.GetPreviousBody())
      return true;
  }
  //Addition 2: for closed chains, check if the pair is closing a loop
  //pair<size_t, size_t> thisPair = make_pair(m_index, _otherBody->m_index);
  for(auto eachPair: m_closureIndices) {
    if(m_index == eachPair.first && _otherBody->m_index == eachPair.second)
      return true;
    if(_otherBody->m_index == eachPair.first && m_index == eachPair.second)
      return true;
  }
  //Addition 3: check branching on the closure connection
  for(auto eachPair: m_closureIndices) {
    if(m_backwardConnections.size() > 0) {
      if(m_index == eachPair.first) {
        if(_otherBody->GetBackwardConnection(0).GetPreviousBody()->m_index == eachPair.second)
          return true;

      }
      if(m_index == eachPair.second) {
        if(_otherBody->GetBackwardConnection(0).GetPreviousBody()->m_index == eachPair.first)
          return true;
      }
      if(_otherBody->m_index == eachPair.first) {
        auto c = m_backwardConnections[0];
        if(c->GetPreviousBody()->m_index == eachPair.second)
          return true;
      }
      if(_otherBody->m_index == eachPair.second) {
        auto c = m_backwardConnections[0];
        if(c->GetPreviousBody()->m_index == eachPair.first)
          return true;
      }
    }
  }

  //4. Addition 4: branched closures
  for(auto eachPair1 : m_closureIndices) {
    if(m_index == eachPair1.first || m_index == eachPair1.second) {
      for(auto eachPair2 : m_closureIndices) {
        if(_otherBody->m_index == eachPair2.first || _otherBody->m_index == eachPair2.second) {
          if(eachPair1.first == eachPair2.first)
            return true;
        }
      }
    }
  }
  return this == _otherBody.get();
}

bool
FreeBody::
IsWithinI(shared_ptr<FreeBody> _otherBody, size_t _i) const {
  return IsWithinI(this, _otherBody.get(), _i, NULL);
}

void
FreeBody::
Link(Connection* _c) {
  m_forwardConnections.push_back(_c);
  _c->GetNextBody()->m_backwardConnections.push_back(_c);
  m_worldPolyhedronAvailable = false;
  m_centerOfMassAvailable = false;
}

Transformation&
FreeBody::
GetWorldTransformation() {
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
Configure(Transformation& _transformation) {
  m_worldTransformation = _transformation;
  m_centerOfMassAvailable = false;
  m_worldPolyhedronAvailable = false;
}

void
FreeBody::
ConfigureRender(Transformation& _transformation) {
  m_renderTransformation = _transformation;
}

void
FreeBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  ReadOptions(_is, _cbs);

#ifdef PMPState
  m_mass = ReadField<double>(_is, _cbs, "Failed reading body mass.");
#endif

  Read(m_comAdjust);

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
        m_worldTransformation =
          ReadField<Transformation>(_is, _cbs,
              "Failed reading fixed based transformation.");
        ConfigureRender(m_worldTransformation);
        break;
      }

      //if the base is a joint nothing additional is parsed
    case BodyType::Joint:
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
      _os << _fb.m_worldTransformation;
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
    FreeBody* next = c->GetNextBody().get();
    if(next != _prevBody && IsWithinI(next, _body2, _i - 1, _body1))
      return true;
  }
  for(const auto& c : m_backwardConnections) {
    FreeBody* prev = c->GetPreviousBody().get();
    if(prev != _prevBody && IsWithinI(prev, _body2, _i - 1, _body1))
      return true;
  }
  return false;
}

Transformation&
FreeBody::
ComputeWorldTransformation(set<size_t>& _visited) {
  m_centerOfMassAvailable = false;
  m_worldPolyhedronAvailable = false;

  if(_visited.find(m_index) != _visited.end()) {
    return m_worldTransformation;
  }
  else {
    _visited.insert(m_index);

    if(m_backwardConnections.empty())
      return m_worldTransformation;

    Connection& back = *m_backwardConnections[0];
    Transformation dh = back.GetDHParameters().GetTransformation();
    m_worldTransformation =
      back.GetPreviousBody()->ComputeWorldTransformation(_visited) *
      back.GetTransformationToDHFrame() *
      dh *
      back.GetTransformationToBody2();

    return m_worldTransformation;
  }
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
    Transformation dh = back.GetDHRenderParameters().GetTransformation();
    m_renderTransformation =
      back.GetPreviousBody()->ComputeRenderTransformation(_visited) *
      back.GetTransformationToDHFrame() *
      dh *
      back.GetTransformationToBody2();

    return m_renderTransformation;
  }
}
