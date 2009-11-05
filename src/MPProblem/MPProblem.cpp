#include "MPProblem.h"
#include "MPStrategy.h"

MPProblem::
MPProblem(XMLNodeReader& in_Node) : MPBaseObject(in_Node, this) {
  LOG_DEBUG_MSG("MPProblem::MPProblem()");
  
  ParseXML(in_Node);
 // rmp.environment = m_pEnvironment;
 // m_pStatClass = new Stat_Class;
  
  LOG_DEBUG_MSG("~MPProblem::MPProblem()");
}


void MPProblem::
ParseXML(XMLNodeReader& in_Node) { 
  LOG_DEBUG_MSG("MPProblem::ParseXML()");

  in_Node.verifyName("MPProblem");

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "environment") {
      m_pEnvironment = new Environment(*citr, this);
    } else  if(citr->getName() == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(*citr, this);
    } else  if(citr->getName() == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
    } 
      else  if(citr->getName() == "validity_test") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
      m_pValidityChecker = new ValidityChecker<CfgType>(*citr, this);
    } else  if(citr->getName() == "MPRegions") {
      ///\Todo Parse MPRegions
    } else  if(citr->getName() == "NeighborhoodFinder") {
      m_pNeighborhoodFinder = new NeighborhoodFinder(*citr,this);
    }else {
      citr->warnUnknownNode();
    }
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
  if( in_RegionId >= m_vecMPRegions.size()) 
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

