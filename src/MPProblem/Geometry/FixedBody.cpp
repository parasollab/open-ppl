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
FixedBody::FixedBody(MultiBody* _owner) :
    Body(_owner)
{
}

FixedBody::FixedBody(MultiBody* _owner, GMSPolyhedron & _polyhedron) :
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


/*
bool FixedBody::operator==(const FixedBody& b) const
{
  return (multibody == b.multibody) && 
         (worldTransformation == b.worldTransformation) &&
         (polyhedron == b.polyhedron) &&
         (worldPolyhedron == b.worldPolyhedron) &&
         (CenterOfMassAvailable == b.CenterOfMassAvailable) &&
         (CenterOfMass == b.CenterOfMass) &&
         (boundingBox[0] == b.boundingBox[0]) &&
         (boundingBox[1] == b.boundingBox[1]) &&
         (boundingBox[2] == b.boundingBox[2]) &&
         (boundingBox[3] == b.boundingBox[3]) &&
         (boundingBox[4] == b.boundingBox[4]) &&
         (boundingBox[5] == b.boundingBox[5]) &&
         (bb_polyhedron == b.bb_polyhedron) &&
         (bb_world_polyhedron == b.bb_world_polyhedron) &&
         (forwardConnection == b.forwardConnection) &&
         (backwardConnection == b.backwardConnection) &&
         (contactCount == b.contactCount); 
}
*/
