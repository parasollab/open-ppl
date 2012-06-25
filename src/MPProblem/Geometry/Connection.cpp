/////////////////////////////////////////////////////////////////////
//  Connection.cpp
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include "Connection.h"
#include "MultiBody.h"


//===================================================================
//  Constructors and Destructor
//===================================================================
Connection::Connection(MultiBody* _owner)
  : m_multibody(_owner) {
}

Connection::Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2)
  : transformationToBody2(Transformation::Identity),
    transformationToDHFrame(Transformation::Identity),
    dhparameters(DHparameters(0.0, 0.0, 0.0, 0.0))
{
  body[0] = _body1;
  body[1] = _body2;
}

Connection::Connection(const shared_ptr<Body>& _body1, const shared_ptr<Body>& _body2, 
                       const Transformation & _transformationToBody2, 
	               const DHparameters & _dhparameters, 
                       const Transformation & _transformationToDHFrame) 
  : transformationToBody2(_transformationToBody2),
    transformationToDHFrame(_transformationToDHFrame),
    dhparameters(_dhparameters)
{
  body[0] = _body1;
  body[1] = _body2;
}

Connection::~Connection() 
{}


//-------------------------------------------------------------------
//  IsFirstBody
//-------------------------------------------------------------------
bool Connection::IsFirstBody(const shared_ptr<Body>& _body) const
{
  return body[0] == _body;
}


//-------------------------------------------------------------------
//  GetPreviousBody
//-------------------------------------------------------------------
shared_ptr<Body> Connection::GetPreviousBody() 
{
  return body[0];
}

//-------------------------------------------------------------------
//  GetNextBody
//-------------------------------------------------------------------
shared_ptr<Body> Connection::GetNextBody() 
{
  return body[1];
}

   
//-------------------------------------------------------------------
//  GetConnectionType
//-------------------------------------------------------------------
Robot::JointType Connection::GetConnectionType() const
{
  return type;
}


//-------------------------------------------------------------------
//  GetDHparameters
//-------------------------------------------------------------------
DHparameters & Connection::GetDHparameters() 
{
  return dhparameters;
}


//-------------------------------------------------------------------
//  GetTransformationToBody2
//-------------------------------------------------------------------
Transformation & Connection::GetTransformationToBody2() 
{
  return transformationToBody2;
}

//-------------------------------------------------------------------
//  GetTransformationToDHFrame
//-------------------------------------------------------------------
Transformation & Connection::GetTransformationToDHFrame() 
{
  return transformationToDHFrame;
}

//===================================================================
//  Write
//===================================================================
ostream&
operator<<(ostream& _os, const Connection& _c){
  return _os << _c.m_prevBodyIdx << " " << _c.m_nextBodyIdx << " "
    << Robot::GetTagFromJointType(_c.type) << endl 
    << _c.transformationToDHFrame << " " << _c.dhparameters << " "
    << _c.transformationToBody2;
}

//===================================================================
//  Read
//===================================================================
istream&
operator>>(istream& _is, Connection& _c){
  //body indices
  _c.m_prevBodyIdx = ReadField<int>(_is, "Previous Body Index");
  _c.m_nextBodyIdx = ReadField<int>(_is, "Next Body Index");
  
  //grab the shared_ptr to bodies
  _c.body[0] = _c.m_multibody->GetFreeBody(_c.m_prevBodyIdx);
  _c.body[1] = _c.m_multibody->GetFreeBody(_c.m_nextBodyIdx);

  //grab the joint type
  string connectionTypeTag = ReadFieldString(_is, "Connection Type");
  _c.type = Robot::GetJointTypeFromTag(connectionTypeTag);

  //transformation to DHFrame
  _c.transformationToDHFrame = 
    ReadField<Transformation>(_is, "Transformation to DH frame");

  //DH parameters
  _c.dhparameters = ReadField<DHparameters>(_is, "DH Parameters");

  //transformation to next body
  _c.transformationToBody2 =
    ReadField<Transformation>(_is, "Transform to next body");

  //make the connection
  _c.body[0]->Link(_c);

  return _is;
}

bool Connection::operator==(const Connection& c) const 
{
  return
    body[0] == c.body[0] &&
    body[1] == c.body[1] &&
    transformationToBody2 == c.transformationToBody2 &&
    transformationToDHFrame == c.transformationToDHFrame &&
    dhparameters == c.dhparameters &&
    type == c.type
    ;
}
