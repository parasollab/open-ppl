#ifndef MPProblem_h
#define MPProblem_h

#include "tinyxml.h"

#include "SwitchDefines.h"
#include<sys/time.h>

#include "OBPRMDef.h"
#include "Roadmap.h"
#include "Input.h"

#include "Clock_Class.h"
#include "Stat_Class.h"
#include "CollisionDetection.h"
#include "ConnectMap.h"
#include "DistanceMetrics.h"
#include "LocalPlanners.h"
#include "GenerateMapNodes.h"

#include "GeneratePartitions.h"

//#include "ExplicitInstantiation.h"

/* util.h defines EXIT used in initializing the environment*/
#include "util.h"
#include "CfgTypes.h"
typedef Cfg_free CfgType;
typedef DefaultWeight WeightType;

class MPProblem
{
public:
  MPProblem(TiXmlNode* in_pNode) {
    cout << "Making a MPProblem." << endl;
    parse_xml(in_pNode);

    rmp.environment = env;
  }
  
  void print_input_options()
  {
    cout << "Parsing MPproblem" << endl;
    cout << "  input_env  = " << m_input_env << endl;
    cout << "  output_map = " << m_output_map << endl;
    cout << "  output_dir = " << m_output_dir << endl;
  }

  string GetEnvFileName() { return m_input_env; }
  
private:
  ///\todo Create constructors for distance_metrics, collision_detection, MPRegions
  void parse_xml(TiXmlNode* in_pNode) { 
    if(!in_pNode) {
      cout << "Error -1" << endl; exit(-1);
    }
    if(string(in_pNode->Value()) != "MPProblem") {
      cout << "Error reading <MPProblem> tag...." << endl; exit(-1);
    }
  
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "file_io") {
        parse_file_io(pChild);
      } else if(string(pChild->Value()) == "environment") {
        cout << "I am making an Environment" << endl;
        env = new Environment(pChild);
      } else  if(string(pChild->Value()) == "distance_metrics") {
        cout << "I am making a <distance_metrics>" << endl;
        dm = new DistanceMetric(pChild);
      } else  if(string(pChild->Value()) == "collision_detection") {
        cd = new CollisionDetection(pChild);
      } else  if(string(pChild->Value()) == "MPRegions") {
        //m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
      }else {
        cout << "I dont know: " << *pChild << endl;
      }
    }

  print_input_options();
}

  void parse_file_io(TiXmlNode* in_pNode) {
    cout << "I am parsing file_io" << endl;
    if(string(in_pNode->Value()) != "file_io") {
      cout << "Error reading <file_io> tag...." << endl; exit(-1);
    }
  
    for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
      if(string(pChild->Value()) == "input_env") {
        m_input_env = string(pChild->ToElement()->Attribute("file_name"));
      } else if(string(pChild->Value()) == "output_map") {
        m_output_map = string(pChild->ToElement()->Attribute("file_name"));
      } else  if(string(pChild->Value()) == "output_dir") {
        m_output_dir = string(pChild->ToElement()->Attribute("dir_name"));
      } else {
        cout << "I dont know: " << *pChild << endl;
      }
    }
   cout << "I am finished parsing file_io" << endl;
  }

////////////
//
//Data
//
//////////////
  public:
 
  string m_input_env, m_output_map, m_output_dir;
  DistanceMetric*     dm;
  CollisionDetection* cd;
  Environment* env;
  Roadmap<CfgType,WeightType> rmp;
  Roadmap<CfgType,WeightType> rmp_col;
  vector< MPRegion<CfgType,WeightType> > regions; 
   
    
};



#endif
