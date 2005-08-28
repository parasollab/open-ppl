#include "MPProblem.h"
#include "MPStrategy.h"

MPProblem::
MPProblem(TiXmlNode* in_pNode) : MPBaseObject(in_pNode, this) {
  LOG_DEBUG_MSG("MPProblem::MPProblem()");
  
  ParseXML(in_pNode);
  rmp.environment = m_pEnvironment;
  m_pStatClass = new Stat_Class;
  
  LOG_DEBUG_MSG("~MPProblem::MPProblem()");
}




void MPProblem::
ParseXML(TiXmlNode* in_pNode) { 
  LOG_DEBUG_MSG("MPProblem::ParseXML()");
  if(!in_pNode) {
    LOG_ERROR_MSG("MPProblem::ParseXML() error xml input"); exit(-1);
  }
  if(string(in_pNode->Value()) != "MPProblem") {
    LOG_ERROR_MSG("MPProblem::ParseXML() error xml input"); exit(-1);
  }

  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; 
       pChild = pChild->NextSibling()) {

    if(string(pChild->Value()) == "file_io") {
      ParseXMLFileIO(pChild);
    } else if(string(pChild->Value()) == "environment") {
      m_pEnvironment = new Environment(pChild, this);
    } else  if(string(pChild->Value()) == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(pChild, this);
    } else  if(string(pChild->Value()) == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(pChild, this);
    } else  if(string(pChild->Value()) == "MPRegions") {
      ///\Todo Parse MPRegions
      //m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
    }else {
      LOG_WARNING_MSG("MPProblem::  I don't know: "<< endl << *pChild);
    }
  }

  LOG_DEBUG_MSG("~MPProblem::ParseXML()");
}

void MPProblem::
ParseXMLFileIO(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("MPProblem::ParseXMLFileIO()");
  if(string(in_pNode->Value()) != "file_io") {
    LOG_ERROR_MSG("MPProblem::ParseFileIO() error xml input"); exit(-1);
  }

  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; 
       pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "input_env") {
      m_input_env = string(pChild->ToElement()->Attribute("file_name"));
    } else if(string(pChild->Value()) == "output_map") {
      m_output_map = string(pChild->ToElement()->Attribute("file_name"));
    } else  if(string(pChild->Value()) == "output_dir") {
      m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
    } else {
      LOG_WARNING_MSG("MPProblem::  I don't know: "<< endl << *pChild);
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

void MPProblem::
WriteRoadmapForVizmo() {
  LOG_DEBUG_MSG("MPProblem::WriteRoadmapForVizmo()");
  ofstream  myofstream(GetOutputRoadmap().c_str());
  
  if (!myofstream) {
    LOG_ERROR_MSG("MPProblem::WriteRoadmapForVizmo: can't open outfile: ");
    exit(-1);
  }
  
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  myofstream << endl << "#####PREAMBLESTART#####";
  myofstream << endl << "../obprm -f " << GetEnvFileName() << " ";//commandLine;
  myofstream << " -bbox ";
  GetEnvironment()->GetBoundingBox()->Print(myofstream, ',', ',');
  myofstream << endl << "#####PREAMBLESTOP#####";
  
  myofstream << endl << "#####ENVFILESTART#####";
  myofstream << endl << GetEnvFileName();
  myofstream << endl << "#####ENVFILESTOP#####";
  
  m_pMPStrategy->GetLocalPlanners()->WriteLPs(myofstream);
  GetCollisionDetection()->WriteCDs(myofstream);
  GetDistanceMetric()->WriteDMs(myofstream);
  GetRoadmap()->WriteRNGseed(myofstream);

  GetRoadmap()->m_pRoadmap->WriteGraph(myofstream);         // writes verts & adj lists
  myofstream.close();
  LOG_DEBUG_MSG("~MPProblem::WriteRoadmapForVizmo()");
}

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
