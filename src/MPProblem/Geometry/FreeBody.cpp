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

  return m_worldPolyhedron;
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
  m_worldTransformation = _transformation;
  m_centerOfMassAvailable=false;
  m_worldPolyhedronAvailable=false;//transformation changed
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
FreeBody::GetWorldTransformation(bool _debug) {
  std::set<int, less<int> > visited;
  return this->ComputeWorldTransformation(visited, _debug);
}

//this will search the vector of the set of all visited nodes,
//if "this" is not the last object in visited, then it will return
//worldTransFormation.
//else it will insert "this" into the set of visited.
Transformation&
FreeBody::ComputeWorldTransformation(std::set<int, less<int> >& visited, bool _debug) {
  m_centerOfMassAvailable=false;
  m_worldPolyhedronAvailable=false;//transformation changed
  if(_debug) {
    cout << "ComputeWorldTransformation::visited =";
    for(std::set<int, less<int> >::const_iterator V = visited.begin(); V != visited.end(); ++V)
      cout << " " << *V;
    cout << endl;
  }
  if(visited.find(m_multibody->GetFreeBodyIndex(*this)) != visited.end()) {
    if(_debug) {
    cout << "index " << m_multibody->GetFreeBodyIndex(*this) << " already visited, transformation is " << m_worldTransformation << ", returning\n";
    }
    return m_worldTransformation;

  } else {
    if(_debug) {
    cout << "index " << m_multibody->GetFreeBodyIndex(*this) << " not visited yet.\n";
    }
    visited.insert(m_multibody->GetFreeBodyIndex(*this));

    //for the case when the base is a freebody.
    if(m_backwardConnection.empty()) {//base link
      if(_debug) {
        cout << "base link, no transformation change, transformation is " << m_worldTransformation << ", returning\n";
      }
      return m_worldTransformation;
    }

    Transformation dh = m_backwardConnection[0].GetDHparameters().GetTransformation();
    m_worldTransformation =
      ((FreeBody*)(m_backwardConnection[0].GetPreviousBody().get()))->ComputeWorldTransformation(visited, _debug)
      * m_backwardConnection[0].GetTransformationToDHFrame()
      * dh
      * m_backwardConnection[0].GetTransformationToBody2();
    if(_debug) {
      cout << "computing new transformation by:\n\tprior transformation: " << ((FreeBody*)(m_backwardConnection[0].GetPreviousBody().get()))->ComputeWorldTransformation(visited, false) << endl;
      cout << "\ttransformation to DH frame: " << m_backwardConnection[0].GetTransformationToDHFrame() << endl;
      cout << "\t\tdh from connection: " << m_backwardConnection[0].GetDHparameters() << endl;
      cout << "\tdh: " << dh << endl;
      cout << "\ttransformation to body 2: " << m_backwardConnection[0].GetTransformationToBody2() << endl;
      cout << "new transformation = " << m_worldTransformation << ", returning\n";
    }

    return m_worldTransformation;
  }
}

istream&
operator>>(istream& _is, FreeBody& _fb){
  _fb.m_filename = ReadFieldString(_is, "FreeBody filename (geometry file)", false);
  _fb.Read();

  //Read for Base Type.  If Planar or Volumetric, read in two more strings
  //If Joint skip this stuff. If Fixed read in positions like an obstacle
  string baseTag = ReadFieldString(_is,
      "Base Tag (Planar, Volumetric, Fixed, Joint");
  _fb.m_baseType = Robot::GetBaseFromTag(baseTag);


  if(_fb.m_baseType == Robot::VOLUMETRIC ||_fb. m_baseType == Robot::PLANAR){
    _fb.m_isBase = true;
    string baseMovementTag = ReadFieldString(_is,
        "Rotation Tag (Rotational, Translational)");
   _fb.m_baseMovementType = Robot::GetMovementFromTag(baseMovementTag);
  }
  else if(_fb.m_baseType == Robot::FIXED){
    _fb.m_isBase = true;
    _fb.m_worldTransformation =
      ReadField<Transformation>(_is, "FreeBody Transformation");
  }

  return _is;
}

ostream&
operator<<(ostream& _os, FreeBody& _fb){
  _os << _fb.m_filename << " ";

  _os << Robot::GetTagFromBase(_fb.m_baseType) << " ";

  if(_fb.m_baseType == Robot::VOLUMETRIC || _fb.m_baseType == Robot::PLANAR){
    _os << Robot::GetTagFromMovement(_fb.m_baseMovementType);
  }
  else if(_fb.m_baseType == Robot::FIXED){
    _os << _fb.m_worldTransformation;
  }

  return _os;
}

