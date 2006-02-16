/** @file main_pmp.cpp
 * @brief main function to use the feature sensitive meta-planner
 */


#include "Input.h"


#include "CfgTypes.h"

#include "Weight.h"


//typedef Cfg_free CfgType;
//typedef DefaultWeight WeightType;

//#include "MetaPlanner.h"
#include "main_pmpl.h"

//using namespace mp;

//========================================================================
//  main
//========================================================================



int main(int argc, char** argv)
{
  
  MPProblem* problem;
  MPStrategy* strategy;
  
  if(argc < 3) { cout << "Usage ... -f options.xml" << endl; exit(-1);}
  
  if(!(string(argv[1]) == "-f"))
  { cout << "Usage ... -f options.xml" << endl; exit(-1);}
  
  TiXmlDocument doc( argv[2] );
  bool loadOkay = doc.LoadFile();
  
  
  if ( !loadOkay ) {
    cout << "Could not load test file " << string(argv[2]) << ". Error=" << doc.ErrorDesc() <<". Exiting.\n";
    exit( 1 );
  }

  TiXmlNode* in_pNode = 0;
  
  in_pNode = doc.FirstChild( "motion_planning" );
  assert( in_pNode );
  
  if(!in_pNode) {
    cout << "Error -1" << endl; exit(-1);
  }
  if(string(in_pNode->Value()) != "motion_planning") {
    cout << "Error reading <motion_planning> tag...." << endl; exit(-1);
  }
  
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling())
  {
    if(pChild->Type() == TiXmlNode::ELEMENT) {
      if(string(pChild->Value()) == "MPProblem") {
          problem = new MPProblem(pChild);
      }
      else if(string(pChild->Value()) == "MPStrategy") {
        strategy = new MPStrategy(pChild,problem);
        problem->SetMPStrategy(strategy);
      }
      else {
        parse_unknown_tag(pChild);
      } 
    }
  }
  //Output whats about to go on
  problem->PrintOptions(cout);
  //Start generating!
  strategy->Solve();
    return 0;
}


