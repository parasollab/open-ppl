#include "MPProblem.h"
#include "MPStrategy.h"

MPProblem::
MPProblem(TiXmlNode* in_pNode) {
  LOG_MSG("MPProblem::MPProblem()",DEBUG_MSG);
  
  ParseXML(in_pNode);
  rmp.environment = m_pEnvironment;
}

void MPProblem::
ParseXML(TiXmlNode* in_pNode) { 
  LOG_MSG("MPProblem::ParseXML()",DEBUG_MSG);
  if(!in_pNode) {
    LOG_MSG("MPProblem::ParseXML() error xml input",ERROR_MSG); exit(-1); exit(-1);
  }
  if(string(in_pNode->Value()) != "MPProblem") {
    LOG_MSG("MPProblem::ParseXML() error xml input",ERROR_MSG); exit(-1);
  }

  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; 
       pChild = pChild->NextSibling()) {

    if(string(pChild->Value()) == "file_io") {
      ParseXMLFileIO(pChild);
    } else if(string(pChild->Value()) == "environment") {
      m_pEnvironment = new Environment(pChild, this);
    } else  if(string(pChild->Value()) == "distance_metrics") {
      m_pDistanceMetric = new DistanceMetric(pChild);
    } else  if(string(pChild->Value()) == "collision_detection") {
      m_pCollisionDetection = new CollisionDetection(pChild);
    } else  if(string(pChild->Value()) == "MPRegions") {
      //m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
    }else {
      LOG_MSG("MPProblem::  I don't know: "<< endl << *pChild,WARNING_MSG);
    }
  }

  PrintOptions();
  LOG_MSG("~MPProblem::ParseXML()",DEBUG_MSG);
}

void MPProblem::
ParseXMLFileIO(TiXmlNode* in_pNode) {
  LOG_MSG("MPProblem::ParseXMLFileIO()",DEBUG_MSG);
  if(string(in_pNode->Value()) != "file_io") {
    LOG_MSG("MPProblem::ParseFileIO() error xml input",ERROR_MSG); exit(-1);
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
      LOG_MSG("MPProblem::  I don't know: "<< endl << *pChild,WARNING_MSG);
    }
  }
  LOG_MSG("~MPProblem::ParseXMLFileIO()",DEBUG_MSG);
}

void MPProblem::
PrintOptions()
{
  cout << "MPProblem" << endl;
  cout << "  input_env  = " << m_input_env << endl;
  cout << "  output_map = " << m_output_map << endl;
  cout << "  output_dir = " << m_output_dir << endl;
}

void MPProblem::
WriteRoadmapForVizmo() {
  LOG_MSG("MPProblem::WriteRoadmapForVizmo()",DEBUG_MSG);
  ofstream  myofstream(GetOutputRoadmap().c_str());
  
  if (!myofstream) {
    LOG_MSG("MPProblem::WriteRoadmapForVizmo: can't open outfile: ",ERROR_MSG);
    exit(-1);
  }
  
  myofstream << "Roadmap Version Number " << RDMPVER_CURRENT_STR;
  myofstream << endl << "#####PREAMBLESTART#####";
  myofstream << endl << "../obprm -f -bbox [-10,10,-10,10,-10,10]";//commandLine;
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
  LOG_MSG("~MPProblem::WriteRoadmapForVizmo()",DEBUG_MSG);
}