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


class MPproblem
{
public:
  MPproblem(TiXmlNode* in_pNode) {
    if(!in_pNode) {
      cout << "Error -1" << endl; exit(-1);
    }
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
    
    print_input_options();
  }
  
  void print_input_options()
  {
    cout << "Parsing MPproblem" << endl;
    cout << "  input_env  = " << m_input_env << endl;
    cout << "  output_map = " << m_output_map << endl;
    cout << "  output_dir = " << m_output_dir << endl;
  }
private:

  string m_input_env, m_output_map, m_output_dir;
  DistanceMetric     dm;
  CollisionDetection cd;
  Environment* env;
  Roadmap<CfgType,WeightType> rmp;
  Roadmap<CfgType,WeightType> rmp_col;
  vector< MPRegion<CfgType,WeightType> > regions; 
   
    
};
