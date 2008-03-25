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
    if(citr->getName() == "file_io") {
      ParseXMLFileIO(*citr);
    } else if(citr->getName() == "environment") {
      m_pEnvironment = new Environment(*citr, this);
    } else  if(citr->getName() == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(*citr, this);
    } else  if(citr->getName() == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(*citr, this);
    } else  if(citr->getName() == "MPRegions") {
      ///\Todo Parse MPRegions
      //m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
    }else {
      citr->warnUnknownNode();
    }
  }

  LOG_DEBUG_MSG("~MPProblem::ParseXML()");
}

void MPProblem::
ParseXMLFileIO(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("MPProblem::ParseXMLFileIO()");
  
  in_Node.verifyName("file_io");
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "input_env") {
      m_input_env = citr->stringXMLParameter("file_name",true,"","file_name");
    } else if(citr->getName() == "output_map") {
      m_output_map = citr->stringXMLParameter("file_name",true,"","file_name");
    } else  if(citr->getName() == "output_dir") {
      m_output_dir = citr->stringXMLParameter("dir_name",true,"","dir_name");
    } else {
      citr->warnUnknownNode();
    }
  }
  LOG_DEBUG_MSG("~MPProblem::ParseXMLFileIO()");
}

void MPProblem::
PrintOptions(ostream& out_os)
{
  out_os << "MPProblem" << endl;
  out_os << "  input_env  = " << m_input_env << endl;
  out_os << "  output_map = " << m_output_map << endl;
  out_os << "  output_dir = " << m_output_dir << endl;
  
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

