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
Connection::Connection(Body * _body1, Body * _body2) {
    body[0] = _body1;
    body[1] = _body2;
    transformationToBody2 = Transformation::Identity;
    transformationToDHFrame = Transformation::Identity;
    dhparameters = DHparameters(0.0, 0.0, 0.0, 0.0);
}

Connection::Connection(Body * _body1, Body * _body2, const Transformation & _transformationToBody2, 
	const DHparameters & _dhparameters, const Transformation & _transformationToDHFrame) {
    body[0] = _body1;
    body[1] = _body2;
    transformationToBody2 = _transformationToBody2;
    transformationToDHFrame = _transformationToDHFrame;
    dhparameters = _dhparameters;
    type = Revolute;
}

Connection::~Connection() {
    body[0]->RemoveForwardConnection(this, 0);
    body[1]->RemoveBackwardConnection(this, 0);
}


//===================================================================
//  Read
//===================================================================
void Connection::Read(Body* body1, Body* body2,
		      const Vector3D& transformPosition, const Orientation& transformOrientation,
		      const Vector3D& positionToDHFrame, const Orientation& orientationToDHFrame,
		      const DHparameters& _dhparameters, const ConnectionType& connectionType) {
  body[0] = body1;
  body[1] = body2;
  
  transformationToBody2 = Transformation(transformOrientation, transformPosition);
  transformationToDHFrame = Transformation(orientationToDHFrame, positionToDHFrame);
  dhparameters = _dhparameters;  
  type = connectionType;
  
#ifndef QUIET
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
#endif 
}


//===================================================================
//  Write
//===================================================================
void Connection::Write(ostream & _os) {
    _os << "Connection" << endl;
    if (body[1]->IsFixedBody()) {
        _os << (int)0 << " ";
	_os << body[1]->GetMultiBody()->GetFixedBodyIndex((FixedBody *)body[1]) << endl;
    } else {
        _os << (int)1 << " ";
	_os << body[1]->GetMultiBody()->GetFreeBodyIndex((FreeBody *)body[1]) << endl;
    }
    transformationToBody2.Write(_os);
    dhparameters.Write(_os);
    _os << (int)type << endl;
}
