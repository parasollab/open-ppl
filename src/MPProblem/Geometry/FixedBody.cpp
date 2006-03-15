// $Id$
/////////////////////////////////////////////////////////////////////
//  FixedBody.c
//
//  Created   3/ 1/98 Aaron Michalk
/////////////////////////////////////////////////////////////////////

#include "FixedBody.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
FixedBody::FixedBody(MultiBody * _owner) :
    worldPolyhedronComputed(0),
    Body(_owner)
{
}

FixedBody::FixedBody(MultiBody * _owner, GMSPolyhedron & _polyhedron) :
    worldPolyhedronComputed(0),
    Body(_owner, _polyhedron)
{
}

FixedBody::~FixedBody() {
}

//===================================================================
//  GetWorldPolyhedron
//
//  Need a mechanism to distinguish between an obstacle and an object
//===================================================================
GMSPolyhedron & FixedBody::GetWorldPolyhedron() {
 //GMSPolyhedron a;
#if 0
    if (!worldPolyhedronComputed) {
        Body::GetWorldPolyhedron();
        worldPolyhedronComputed = 1;
    }
#endif
   GetWorldTransformation();	///<----?????
   worldPolyhedron=Body::GetWorldPolyhedron();
   return worldPolyhedron;
}

//===================================================================
//  GetWorldTransformation
//===================================================================
Transformation & FixedBody::GetWorldTransformation() {
    return worldTransformation;
}

#if 0
//===================================================================
//  Configure
//  
//  Function: Configure "this" body with the given transformation
//
//  For a fixed body, there is only one transformation that need to be 
//  taken care of
//===================================================================
void FixedBody::Configure(Transformation & _transformation){
    // new transformation (position and orientation) for the reconfiguration
    worldTransformation = _transformation;
}
#endif

//===================================================================
//  Write               
//===================================================================
void FixedBody::Write(ostream & _os) {
    _os << "FixedBody" << endl;
    Body::Write(_os);
    worldTransformation.Write(_os);
    _os << endl;
}

