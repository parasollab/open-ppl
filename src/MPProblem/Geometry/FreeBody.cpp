#include "FreeBody.h"

#include "ActiveMultiBody.h"

FreeBody::
FreeBody(MultiBody* _owner) : Body(_owner) {
}

FreeBody::
FreeBody(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  Body(_owner, _polyhedron){
  }

void
FreeBody::
Read(istream& _is, CountingStreamBuffer& _cbs) {

  m_filename = ReadFieldString(_is, _cbs,
      "Failed reading geometry filename.", false);

  Read();

  //Read for Base Type.  If Planar or Volumetric, read in two more strings
  //If Joint skip this stuff. If Fixed read in positions like an obstacle
  string baseTag = ReadFieldString(_is, _cbs, "Failed reading base tag."
      " Options are: planar, volumetric, fixed, or joint.");
  m_baseType = GetBaseFromTag(baseTag, _cbs.Where());

  switch(m_baseType) {
    //if base is volumetric or planar we should parse the rotational type
    case VOLUMETRIC:
    case PLANAR:
      {
        m_isBase = true;
        string baseMovementTag = ReadFieldString(_is, _cbs,
            "Failed reading rotation tag."
            " Options are: rotational or translational.");
        m_baseMovementType =
          GetMovementFromTag(baseMovementTag, _cbs.Where());
        break;
      }

    //if base if fixed we should read a transformation
    case FIXED:
      {
        m_isBase = true;
        m_worldTransformation =
          ReadField<Transformation>(_is, _cbs,
              "Failed reading fixed based transformation.");
        break;
      }

    //if the base is a joint nothing additional is parsed
    case JOINT:
      break;
  }
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
}

ostream&
operator<<(ostream& _os, FreeBody& _fb){
  _os << _fb.m_filename << " ";

  _os << Body::GetTagFromBase(_fb.m_baseType) << " ";

  if(_fb.m_baseType == Body::VOLUMETRIC || _fb.m_baseType == Body::PLANAR)
    _os << Body::GetTagFromMovement(_fb.m_baseMovementType);
  else if(_fb.m_baseType == Body::FIXED)
    _os << _fb.m_worldTransformation;

  return _os;
}

