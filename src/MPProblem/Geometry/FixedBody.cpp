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
  m_worldPolyhedron = Body::GetWorldPolyhedron();
  return m_worldPolyhedron;
}

//===================================================================
//  GetWorldTransformation
//===================================================================
Transformation&
FixedBody::GetWorldTransformation() {
  return m_worldTransformation;
}

ostream& 
operator<<(ostream& _os, const FixedBody& _fb){
  return _os << _fb.m_filename << " " << _fb.m_worldTransformation;
}

istream& 
operator>>(istream& _is, FixedBody& _fb){
  _fb.m_filename = ReadFieldString(_is, 
      "FixedBody Filename (geometry file)", false);
  VerifyFileExists(_fb.m_filename);
  _fb.Read(_fb.m_filename);
  _fb.m_worldTransformation = 
    ReadField<Transformation>(_is, "FixedBody Transformation");
  return _is;
}
