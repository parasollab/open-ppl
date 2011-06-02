#include "MPProblem.h"
#include "MPStrategy.h"
#include "DistanceMetrics.h"
#include "ValidityChecker.hpp"
#include "MPRegion.h"


MPProblem::
MPProblem(XMLNodeReader& in_Node, bool parse_xml) : MPBaseObject(in_Node, this) {
  LOG_DEBUG_MSG("MPProblem::MPProblem()");
  
  if(parse_xml)
    ParseXML(in_Node);
 // rmp.environment = m_pEnvironment;
 // m_pStatClass = new Stat_Class;
  
  LOG_DEBUG_MSG("~MPProblem::MPProblem()");
}

MPProblem::
MPProblem(Environment* _m_pEnvironment, DistanceMetric* _m_pDistanceMetric, CollisionDetection* _m_pCollisionDetection, ValidityChecker<CfgType>* _m_pValidityChecker, NeighborhoodFinder* _m_pNeighborhoodFinder) : m_pEnvironment(_m_pEnvironment), m_pDistanceMetric(_m_pDistanceMetric), m_pCollisionDetection(_m_pCollisionDetection), m_pValidityChecker(_m_pValidityChecker), m_pNeighborhoodFinder(_m_pNeighborhoodFinder) {};


bool MPProblem::
ParseChild(XMLNodeReader::childiterator citr)
{
  if(citr->getName() == "environment") {
    m_pEnvironment = new Environment(*citr, this);
    return true;
  } else  if(citr->getName() == "distance_metrics") {
    m_pDistanceMetric = new DistanceMetric(*citr, this);
    return true;
  } else  if(citr->getName() == "collision_detection") {
    m_pCollisionDetection = new CollisionDetection(*citr, this);
    return true;
  } else  if(citr->getName() == "validity_test") {
    m_pCollisionDetection = new CollisionDetection(*citr, this);
    m_pValidityChecker = new ValidityChecker<CfgType>(*citr, this);
    return true;
  } else  if(citr->getName() == "MPRegions") {
    ///\Todo Parse MPRegions
    return true;
  } else  if(citr->getName() == "NeighborhoodFinder") {
    m_pNeighborhoodFinder = new NeighborhoodFinder(*citr,this);
    return true;
  } else
    return false;
}


void MPProblem::
ParseXML(XMLNodeReader& in_Node) { 
  LOG_DEBUG_MSG("MPProblem::ParseXML()");

  in_Node.verifyName("MPProblem");

  for(XMLNodeReader::childiterator citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(!ParseChild(citr))
      citr->warnUnknownNode();
  }

  vector<cd_predefined> cdtypes = m_pCollisionDetection->GetSelectedCDTypes();
  for(vector<cd_predefined>::iterator C = cdtypes.begin(); C != cdtypes.end(); ++C)
    m_pEnvironment->buildCDstructure(*C, 1);

  LOG_DEBUG_MSG("~MPProblem::ParseXML()");
}


void MPProblem::
PrintOptions(ostream& out_os)
{
  out_os << "MPProblem" << endl;
  m_pMPStrategy->PrintOptions(out_os);
  m_pDistanceMetric->PrintOptions(out_os);
  m_pCollisionDetection->PrintOptions(out_os);
  m_pEnvironment->PrintOptions(out_os);
}


int MPProblem::
CreateMPRegion() {
  int returnVal = m_vecMPRegions.size();
  m_vecMPRegions.push_back(new MPRegion<CfgType,WeightType>(returnVal,this));
  return returnVal;
}


MPRegion<CfgType,WeightType>* 
MPProblem::
GetMPRegion(int in_RegionId) {
  if( in_RegionId >= (int)m_vecMPRegions.size()) 
  {  
    LOG_ERROR_MSG("MPProblem:: I dont have region id = " << in_RegionId);
    exit(-1);
  }
  
  return (m_vecMPRegions[in_RegionId]);
}


/*
void MPProblem::
AddToRoadmap(vector<Cfg_free >& in_Cfgs) {

  vector< Cfg_free >::iterator I;
  for(I=in_Cfgs.begin(); I!=in_Cfgs.end(); I++) {
    if((*I).IsLabel("VALID")) {  
      if((*I).GetLabel("VALID")) {//Add to Free roadmap
        rmp.m_pRoadmap->AddVertex((*I));
      } else {  //Add to Coll Roadmap 
      rmp_col.m_pRoadmap->AddVertex((*I));
    }
   }
  }
}
*/

