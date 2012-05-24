// $Id$
/////////////////////////////////////////////////////////////////////
//  Connection.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include "Connection.h"
#include "MultiBody.h"


//===================================================================
//  Constructors and Destructor
//===================================================================
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
//  Read
//===================================================================
void Connection::Read(shared_ptr<Body>& body1, shared_ptr<Body>& body2,
		      const Vector3D& transformPosition, const Orientation& transformOrientation,
		      const Vector3D& positionToDHFrame, const Orientation& orientationToDHFrame,
		      const DHparameters& _dhparameters, const Robot::JointType& connectionType,
                      bool _debug) 
{
  body[0] = body1;
  body[1] = body2;
  
  transformationToBody2 = Transformation(transformOrientation, transformPosition);
  transformationToDHFrame = Transformation(orientationToDHFrame, positionToDHFrame);
  dhparameters = _dhparameters;  
  type = connectionType;
  
  if(_debug) {
    cout << "transformationToBody2 = (" 
         << transformationToBody2.position.getX() << ", " 
         << transformationToBody2.position.getY() << ", " 
         << transformationToBody2.position.getZ() << ", " 
         << transformationToBody2.orientation.alpha << ", " 
         << transformationToBody2.orientation.beta << ", " 
         << transformationToBody2.orientation.gamma << ")" << endl;
    
    cout << "dhparameters = (" 
         << dhparameters.alpha << ", " 
         << dhparameters.a << ", " 
         << dhparameters.d << ", " 
         << dhparameters.theta << ")" 
         << endl;
  }
}


//===================================================================
//  Write
//===================================================================
void Connection::Write(ostream & _os) 
{
  _os << "Connection" << endl;
  if(body[1]->IsFixedBody()) 
  {
    _os << (int)0 << " ";
    _os << body[1]->GetMultiBody()->GetFixedBodyIndex(*(FixedBody*)(body[1].get())) << endl;
  } 
  else 
  {
    _os << (int)1 << " ";
    _os << body[1]->GetMultiBody()->GetFreeBodyIndex(*(FreeBody*)(body[1].get())) << endl;
  }
  transformationToBody2.Write(_os);
  _os << dhparameters;
  _os << (int)type << endl;
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
