///////////////////////////////////////
//
// CarQueryReq.c
//
// derived classs of BasicQueryReq and
// DefaultQueryReqFactory to implement
// query requirements specific for
// car-like robots
//
///////////////////////////////////////

#include "CarQueryReq.h"


///////////////////////////////////////
//
// CarQueryReq
//
///////////////////////////////////////
CarQueryReq::CarQueryReq() : BasicQueryReq() {
  checkTurningRadius = 0;
  turningRadius = 0;

  checkCollision = 0;
}


void
CarQueryReq::init(char * filename) {
 // read query requiremtents from a file.
  
  ifstream  is(filename);

  if (!is) {
    cout << endl << "In CarQueryRequirements: can't open requirements file: " 
         << filename << endl;
    exit(10);
  }

  //read in resolution requirements:
  is >> checkResolution;
  if (checkResolution) {
    is >> resolution;
  }

  //read in clearance requirements:
  is >> checkClearance;
  if (checkClearance) {
    is >> clearance;
  }

  //read in rotation requirements:
  is >> checkRotation;
  if (checkRotation) {
    is >> rotation;
  }

  is >> checkTilting;
  if(checkTilting) {
    is >> minTilt >> maxTilt;
    minTilt = minTilt*M_PI/180;
    maxTilt = maxTilt*M_PI/180; // convert it from drgree to Radian.
  }    
	
  //read in potiential requirements:
  is >> checkPotential;
  if (checkPotential) {
    is >> potential;
  }
  
  is >> checkTurningRadius;
  if (checkTurningRadius)
     is >> turningRadius;

  is >> checkCollision;
  is.close();
  return;
}


bool 
CarQueryReq::nodeValid(Cfg &node, Environment *env, 
			   CollisionDetection *cd, SID cdsetid) {
  if(checkCollision) {
     CDInfo info;
     if(node.isCollision(env, cd, cdsetid, info))
        return false;
  }

  return BasicQueryReq::nodeValid(node, env, cd, cdsetid);
}


void 
CarQueryReq::Print(ostream& os) {
  BasicQueryReq::Print(os);
  os <<   "\tTurning Radius: " << checkTurningRadius << " (" << turningRadius << ")";
  os << "\n\tCollision: "      << checkCollision     << "\n" << flush;
}


bool
CarQueryReq::CheckTurningRadius() {
  return checkTurningRadius;
}


double
CarQueryReq::GetTurningRadius() {
  return turningRadius;
}

bool
CarQueryReq::CheckCollision() {
  return checkCollision;
}


///////////////////////////////////////
//
// CarQueryReqFactory
//
///////////////////////////////////////
bool 
CarQueryReqFactory::Create(IQueryReq ** ppIQueryReq) const {
  //check input
  if( ppIQueryReq==NULL) return false;
  *ppIQueryReq = new CarQueryReq();
  if( *ppIQueryReq==NULL) return false; //not enough memory
  return true;
}
