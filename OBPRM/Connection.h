// $Id$
////////////////////////////////////////////////////////////////////
//  Connection.h
//
//  Created   2/25/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#ifndef Connection_h
#define Connection_h

#include <fstream.h>
#include "Input.h"
#include "DHparameters.h"
#include "Transformation.h"

class Input;
class Body;

class Connection {
public:
    //---------------------------------------------------------------
    //  Enumerations
    //---------------------------------------------------------------
    enum ConnectionType {
        Revolute,
	Prismatic
    };
    //---------------------------------------------------------------
    //  Constructors and Destructor
    //---------------------------------------------------------------
    Connection(Body * _body1, Body * _body2 = 0);  // Second argument is optional
    Connection(Body * _body1, Body * _body2, const Transformation & _transformationToBody2, 
	 const DHparameters & _dhparameters, const Transformation & _transformationToDHFrame);
    ~Connection();
    //---------------------------------------------------------------
    //  Methods
    //---------------------------------------------------------------
    int IsFirstBody(Body * _body);
    Body * GetNextBody();
    ConnectionType GetConnectionType();
    Body * GetPreviousBody();
    DHparameters & GetDHparameters();
    Transformation & GetTransformationToBody2();
    Transformation & GetTransformationToDHFrame();
    void Get(Input * _input, int _multibodyIndex, int _connectionIndex);
    virtual void Write(ostream & _os);
protected:
private:
    //---------------------------------------------------------------
    //  Data
    //---------------------------------------------------------------
    Body *body[2];
    Transformation transformationToBody2;
    Transformation transformationToDHFrame;
    DHparameters dhparameters;
    ConnectionType type;
    int IsActuator;
};

//===================================================================
//  Inline Functions
//===================================================================

//-------------------------------------------------------------------
//  GetDHparameters
//-------------------------------------------------------------------
inline DHparameters & Connection::GetDHparameters() {
    return dhparameters;
}

//-------------------------------------------------------------------
//  IsFirstBody
//-------------------------------------------------------------------
inline int Connection::IsFirstBody(Body * _body) {
    return body[0] == _body;
}

//-------------------------------------------------------------------
//  GetNextBody
//-------------------------------------------------------------------
inline Body * Connection::GetNextBody() {
    return body[1];
}

//-------------------------------------------------------------------
//  GetPreviousBody
//-------------------------------------------------------------------
inline Body * Connection::GetPreviousBody() {
    return body[0];
}

//-------------------------------------------------------------------
//  GetTransformationToBody2
//-------------------------------------------------------------------
inline Transformation & Connection::GetTransformationToBody2() {
    return transformationToBody2;
}

//-------------------------------------------------------------------
//  GetTransformationToDHFrame
//-------------------------------------------------------------------
inline Transformation & Connection::GetTransformationToDHFrame() {
    return transformationToDHFrame;
}

//-------------------------------------------------------------------
//  GetConnectionType
//-------------------------------------------------------------------
inline Connection::ConnectionType Connection::GetConnectionType() {
    return type;
}

#endif


