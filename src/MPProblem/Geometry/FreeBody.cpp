/////////////////////////////////////////////////////////////////////
//  FreeBody.cpp
/////////////////////////////////////////////////////////////////////

#include "FreeBody.h"
#include "MultiBody.h"

//===================================================================
//  Constructors and Destructor
//===================================================================
FreeBody::FreeBody(MultiBody* _owner) : Body(_owner){
}

FreeBody::FreeBody(MultiBody* _owner, GMSPolyhedron& _polyhedron) :
  Body(_owner, _polyhedron){
  }

FreeBody::~FreeBody() {
}

//===================================================================
//  GetWorldPolyhedron
//===================================================================
GMSPolyhedron&
FreeBody::GetWorldPolyhedron() { 
  // Perform a transformation w.r.t. the world frame, and put into the data field
  // of "this" body's instance
  // GetWorldTransformation();

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
void
FreeBody::Configure(Transformation& _transformation){
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
//  Refer to a seperate diagram for the transformation structure
//
//===================================================================
Transformation& 
FreeBody::GetWorldTransformation() {
  std::set<int, less<int> > visited;
  return this->ComputeWorldTransformation(visited);
}

//this will search the vector of the set of all visited nodes, 
//if "this" is not the last object in visited, then it will return
//worldTransFormation.
//else it will insert "this" into the set of visited.
Transformation& 
FreeBody::ComputeWorldTransformation(std::set<int, less<int> >& visited) {
  if(visited.find(multibody->GetFreeBodyIndex(*this)) != visited.end()) {
    return worldTransformation;

  } else {
    visited.insert(multibody->GetFreeBodyIndex(*this));

    //for the case when the base is a freebody.
    if(backwardConnection.empty()) //base link
      return worldTransformation;

    Transformation dh(backwardConnection[0].GetDHparameters());
    worldTransformation =
      ((FreeBody*)(backwardConnection[0].GetPreviousBody().get()))->ComputeWorldTransformation(visited)
      * backwardConnection[0].GetTransformationToDHFrame()
      * dh
      * backwardConnection[0].GetTransformationToBody2();

    return worldTransformation;
  }
}

istream&
operator>>(istream& _is, FreeBody& _fb){
  _fb.m_filename = ReadFieldString(_is, "FreeBody filename (geometry file)", false);

  VerifyFileExists(_fb.m_filename);
  _fb.Read(_fb.m_filename);

  //Read for Base Type.  If Planar or Volumetric, read in two more strings
  //If Joint skip this stuff. If Fixed read in positions like an obstacle
  string baseTag = ReadFieldString(_is, 
      "Base Tag (Planar, Volumetric, Fixed, Joint");
  _fb.baseType = Robot::GetBaseFromTag(baseTag);


  if(_fb.baseType == Robot::VOLUMETRIC ||_fb. baseType == Robot::PLANAR){
    _fb.isBase = true;
    string baseMovementTag = ReadFieldString(_is, 
        "Rotation Tag (Rotational, Translational)");
   _fb.baseMovementType = Robot::GetMovementFromTag(baseMovementTag);
  }
  else if(_fb.baseType == Robot::FIXED){
    _fb.isBase = true;
    _fb.worldTransformation = 
      ReadField<Transformation>(_is, "FreeBody Transformation");
  }

  return _is;
}

ostream&
operator<<(ostream& _os, FreeBody& _fb){
  _os << _fb.m_filename << " ";
  
  _os << Robot::GetTagFromBase(_fb.baseType) << " ";

  if(_fb.baseType == Robot::VOLUMETRIC || _fb.baseType == Robot::PLANAR){
    _os << Robot::GetTagFromMovement(_fb.baseMovementType);   
  }
  else if(_fb.baseType == Robot::FIXED){
    _os << _fb.worldTransformation;   
  }

  return _os;
}

