// $Id$
/////////////////////////////////////////////////////////////////////
//  FreeBody.c
//
//  Created   3/ 1/98 Aaron Michalk
//  Modified  4/13/98 Aaron Michalk
//  Added/Modified  7/31/98 Wookho Son
//  1/13/99 Guang Song GetWorldTransformation method is modified.
//  2/27/99 add WorldTransformation() method.
/////////////////////////////////////////////////////////////////////

#include "FreeBody.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
FreeBody::FreeBody(MultiBody * _owner) :
    Body(_owner)
{
}

FreeBody::FreeBody(MultiBody * _owner, GMSPolyhedron & _polyhedron) :
    Body(_owner, _polyhedron)
{
}

FreeBody::~FreeBody() {
}

//===================================================================
//  GetWorldPolyhedron
//===================================================================
GMSPolyhedron & FreeBody::GetWorldPolyhedron() { 
    // Perform a transformation w.r.t. the world frame, and put into the data field
    // of "this" body's instance
//    GetWorldTransformation();

    // Compute the world-transformed polyhedron and put into the data field
    // of "this" body's instance
    Body::GetWorldPolyhedron();

    return worldPolyhedron;
}

//===================================================================
//  Configure
//  
//  Function: Configure "this" body with the given transformation
//
//  For a single free body, there is only one transformation that need to be 
//  taken care of
//===================================================================
void FreeBody::Configure(Transformation & _transformation){
    // new transformation (position and orientation) for the reconfiguration
    worldTransformation = _transformation;
}

//===================================================================
//  GetWorldTransformation
//
//  Function: Transformation "this" body w.r.t the world frame in a 
//            recursive manner; multiply the world transformation
//            of the previous body with the transformation from the
//            proximal joint to the center of gravity of "this" body
//            (Need a generalization for the connectionship, since
//            currently it handles only one backward connection).
//  
//  Output: The transformation transformed w.r.t the world frame
//
//  Refer to a seperate digram for the transformation structure
//
//===================================================================
Transformation & FreeBody::GetWorldTransformation() {
    //for the case when the base is a freebody.
    if(backwardConnectionCount == 0)  // base link
	return worldTransformation;
    
    ///////////////////////////////////////////////////////////////////////// 
/*
    Connection * lastconn = backwardConnection[0]->GetPreviousBody()->GetBackwardConnection(0);
    Transformation tlastbody;
    if (lastconn)
        tlastbody = lastconn->GetTransformationToBody2().Inverse();
    else
        tlastbody = Transformation::Identity;

    Transformation dh(backwardConnection[0]->GetDHparameters());
    worldTransformation = 
        backwardConnection[0]->GetPreviousBody()->GetWorldTransformation()
        * tlastbody
        * dh
        * backwardConnection[0]->GetTransformationToBody2();
*/

    Transformation dh(backwardConnection[0]->GetDHparameters());
    worldTransformation =
        backwardConnection[0]->GetPreviousBody()->GetWorldTransformation()
        * backwardConnection[0]->GetTransformationToDHFrame()
        * dh
        * backwardConnection[0]->GetTransformationToBody2();

    return worldTransformation;
}

//===================================================================
//  Read                  
//===================================================================
void FreeBody::Get(Input * _input, int _multibodyIndex, int _index) {
    int bodyIndex = _input->BodyIndex[_multibodyIndex][_index];

    Body::Read(_input,_input->freebodyFileName[_multibodyIndex][bodyIndex]);
}

//===================================================================
//  Write               
//===================================================================
void FreeBody::Write(ostream & _os) {
    _os << "FreeBody" << endl;
    Body::Write(_os);
    _os << endl;
}
