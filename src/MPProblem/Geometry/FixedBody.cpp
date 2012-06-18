#include "FixedBody.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
FixedBody::FixedBody(MultiBody* _owner) :
  Body(_owner){
}

FixedBody::FixedBody(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  Body(_owner, _polyhedron){
}

FixedBody::~FixedBody() {
}

//===================================================================
//  GetWorldPolyhedron
//
//  Need a mechanism to distinguish between an obstacle and an object
//===================================================================
GMSPolyhedron&
FixedBody::GetWorldPolyhedron() {
  GetWorldTransformation(); 
  worldPolyhedron = Body::GetWorldPolyhedron();
  return worldPolyhedron;
}

//===================================================================
//  GetWorldTransformation
//===================================================================
Transformation&
FixedBody::GetWorldTransformation() {
  return worldTransformation;
}

//===================================================================
//  Write               
//===================================================================
void
FixedBody::Write(ostream& _os) {
  _os << "FixedBody 0 " << endl;
  Body::Write(_os);
  worldTransformation.Write(_os);
  _os << endl;
}

