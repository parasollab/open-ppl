////////////////////////////////////////////////////
//
//  QueryRequirements.c
//
////////////////////////////////////////////////////


#include "QueryRequirements.h"


////////////////////////////////////////////////////
//
//  BasicQueryReq
//
////////////////////////////////////////////////////
BasicQueryReq::BasicQueryReq() {
  checkResolution = 0;
  resolution = 0;

  checkClearance = 0;
  clearance = 0;

  checkRotation = 0;
  rotation = 0;

  checkTilting = 0;
  minTilt = 0;
  maxTilt = 0;

  checkPotential = 0;
  potential = 0;
}


void 
BasicQueryReq::init(char* filename) {
  // read query requiremtents from a file.
  
  ifstream  is(filename);

  if (!is) {
    cout << endl << "In QueryRequirements: can't open requirements file: " 
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

  is.close();
  return;
}


bool 
BasicQueryReq::nodeValid(Cfg& node, Environment* env,
			 CollisionDetection* cd, SID cdsetid) {
  double d;

  if (checkClearance) {
    //calculate workspace clearance
    d = node.Clearance(env, cd);
    if ((d < clearance) || (d == -1)) { //clearance too small or invalid
      return false;
    }
  }

  if (checkPotential) { //assumes ApproxCSpaceClearance has been 
                         //overwritten to calculate potiential
    //calculate potiential
    d = node.Clearance(env, cd);
    if (d >= potential) { //potiential too large
      return false;
    }
  }

  if (checkTilting) {
      vector<double> tmp = node.GetData();
      double gamma = tmp[3]*M_PI*2;
      double beta = tmp[4]*M_PI*2;
      double ceta = acos(cos(gamma)*cos(beta));
      //cout << "ceta is " << ceta*180/M_PI << "titleAngle is " <<  minTilt*180/M_PI 
      // 	   << "   " << maxTilt*180/M_PI << endl;
      if(ceta > maxTilt || ceta < minTilt )
         return false;
  }

  return true;
}


bool
BasicQueryReq::edgeValid(vector<Cfg> &cfgs, Environment* env,
			 CollisionDetection* cd, SID cdsetid) {
   for(int i=0; i<cfgs.size(); ++i) {
	if( !nodeValid(cfgs[i], env, cd, cdsetid) )
	   return false;
   }
   
   // checking rotation
   if(checkRotation) {
      Cfg incr = cfgs[1] - cfgs[0];
      if(incr.OrientationMagnitude() > rotation) 
	  return false;
   }	  
	  
   return true;
}


void 
BasicQueryReq::Print(ostream& os) {
  os << "\n\tResolution: " << checkResolution << " (" << resolution << ")";
  os << "\n\tClearance: "  << checkClearance  << " (" << clearance  << ")";
  os << "\n\tRotation: "   << checkRotation   << " (" << rotation   << ")";
  os << "\n\tTilting: "    << checkTilting    << " (" << minTilt    << ", " << maxTilt << ")";
  os << "\n\tPotential: "  << checkPotential  << " (" << potential  << ")";
}


////////////////////////////////////////////////////
//
//  DefaultQueryReqFactory
//
////////////////////////////////////////////////////
bool 
DefaultQueryReqFactory::Create(IQueryReq ** ppIQueryReq) const {
  //check input
  if( ppIQueryReq==NULL ) return false;
  *ppIQueryReq = new BasicQueryReq();
  if( *ppIQueryReq==NULL ) return false; //not enough memory
  return true;
}


////////////////////////////////////////////////////
//
//  QueryRequirementsObject
//
////////////////////////////////////////////////////
DefaultQueryReqFactory* QueryRequirementsObject::m_pFactory = new DefaultQueryReqFactory();

QueryRequirementsObject::QueryRequirementsObject() {
  assert(m_pFactory!=NULL);
  bool bResult = m_pFactory->Create(&m_pIQueryReq);
  assert(bResult);
}


QueryRequirementsObject::QueryRequirementsObject(char* filename){
  assert(m_pFactory!=NULL);
  bool bResult = m_pFactory->Create(&m_pIQueryReq);
  assert(bResult);
  m_pIQueryReq->init(filename);
  //PrintValues(cout);
}


IQueryReq* 
QueryRequirementsObject::GetIQueryReq() {
  return m_pIQueryReq;
}


const IQueryReq* 
QueryRequirementsObject::GetIQueryReq() const {
  return m_pIQueryReq;
}


const DefaultQueryReqFactory* 
QueryRequirementsObject::GetDefaultQueryReqFactory() {
  return m_pFactory;
}


void 
QueryRequirementsObject::SetQueryReqFactory(DefaultQueryReqFactory* fact) {
  if( m_pFactory!=NULL )
    delete m_pFactory;
  m_pFactory = fact;
}


bool 
QueryRequirementsObject::isNodeValid(Cfg& node, Environment* env, CollisionDetection* cd, SID cdsetid) {
  return m_pIQueryReq->nodeValid(node, env, cd, cdsetid);
}


bool 
QueryRequirementsObject::isEdgeValid(vector<Cfg>& cfgs, Environment* env, CollisionDetection* cd, SID cdsetid) {
  return m_pIQueryReq->edgeValid(cfgs, env, cd, cdsetid);
}


void 
QueryRequirementsObject::PrintValues(ostream& os) {
  os << "\nQuery Requirements: ";
  m_pIQueryReq->Print(os);
}
