// $Id$
/////////////////////////////////////////////////////////////////////
//  FixedBody.c
//
//  Created   3/ 1/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
//  Added/Modified  6/ 4/98 Wookho Son
//  Added/Modified  7/31/98 Wookho Son
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
//  Modified  6/4/98
//===================================================================
GMSPolyhedron & FixedBody::GetWorldPolyhedron() {
 GMSPolyhedron a;
#if 0
    if (!worldPolyhedronComputed) {
        Body::GetWorldPolyhedron();
        worldPolyhedronComputed = 1;
    }
#endif
   GetWorldTransformation();
    worldPolyhedron=Body::GetWorldPolyhedron();
    //a=worldPolyhedron;
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
//
//  Added  6/4/98    Wookho Son
//===================================================================
void FixedBody::Configure(Transformation & _transformation){
    // new transformation (position and orientation) for the reconfiguration
    worldTransformation = _transformation;
}
#endif

//===================================================================
//  Get
//  Modified  7/22/98  Wookho Son
//===================================================================
void FixedBody::Get(Input * _input, int _multibodyIndex, int _index) {
    // Invoke the "Read" of the parent class
    // (Remember "Read" is a virtual)
    int bodyIndex = _input->BodyIndex[_multibodyIndex][_index];

    //if (_index==0){  // the very first body
    // 6 parameters for world transformation (position and orientation)
    // Now for every fixedbody, see Input.c for corresponding changes. 10/15/99 Guang
    worldTransformation.orientation = _input->fixedbodyOrientation[_multibodyIndex][bodyIndex];
    worldTransformation.position = _input->fixedbodyPosition[_multibodyIndex][bodyIndex];
    //}
    Body::Read(_input,_input->fixedbodyFileName[_multibodyIndex][bodyIndex]);
}

//===================================================================
//  Write               
//===================================================================
void FixedBody::Write(ostream & _os) {
    _os << "FixedBody" << endl;
    Body::Write(_os);
    worldTransformation.Write(_os);
    _os << endl;
}

