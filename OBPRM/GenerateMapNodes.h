//////////////////////////////////////////////////////////////////////////////////////////
/**@file GenerateMapNodes.h
  *This set of classes supports a "RoadMap Node Generation Algobase".
  *Generate roadmap nodes and enter each as a graph node.
  *
  *The classes in the set are:
  * -# GenerateMapNodes
  *
  *
  *@author Lucia K. Dale
  *@date   8/27/98
  * 3/19/03  Shawna  made templated
  */
//////////////////////////////////////////////////////////////////////////////////////////

#ifndef GenerateMapNodes_h
#define GenerateMapNodes_h

//////////////////////////////////////////////////////////////////////////////////////////
// Include OBPRM headers
#include "CollisionDetection.h"
#include "Clock_Class.h"
#include "Parameters.h"
#include "GMSPolyhedron.h"
 
//Include node generation methods
#include "BasicPRM.h"
#include "GaussPRM.h"
#include "CSpaceMAPRM.h"
#include "BasicMAPRM.h"
#include "BasicOBPRM.h"
#include "OBPRM.h"
#include "OBMAPRM.h"
#include "util.h"

#include <sstream>

//////////////////////////////////////////////////////////////////////////////////////////
class Body;
template <class CFG, class WEIGHT> class Roadmap;

//////////////////////////////////////////////////////////////////////////////////////////

#define MAX_NODES          5000000
#define MAX_NODES_PER_OBST 5000



//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class GenerateMapNodes
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This is the main generator class. It contains two vectors: all and selected.
 *all contains all of the different types of node generation methods.  selected
 *contains only those selected by the user.
 */
template <class CFG> class NodeGenerationMethod;


template <class CFG>
class GenerateMapNodes : public MPBaseObject{
 public:

  //////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Constructors and Destructor
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**@name Constructors and Destructor*/
  //@{

  ///Default Constructor.
  GenerateMapNodes();
  GenerateMapNodes(TiXmlNode* in_pNode);
  ///Destructor.	
  ~GenerateMapNodes();

  //@}

  //////////////////////
  // Access methods
  static vector<NodeGenerationMethod<CFG>*> GetDefault();

  //////////////////////
  // I/O methods
  int ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);
  

  /**Generate nodes according to those in selected vector.
   *@param _rm New created nodes will be added to this roadmap if addNodes@Map=true.
   *@param nodes New created nodes are stored here.
   */
  template <class WEIGHT>
  void GenerateNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		     CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes);

 //protected:
 public:
  //////////////////////
  // Data
  vector<NodeGenerationMethod<CFG>*> all;
 public:
  vector<NodeGenerationMethod<CFG>*> selected; //move back to private later

  // public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**Used for record collsion deteciotion inforamtion.
   *This is used in like Cfg::ApproxCSpaceClearance
   *, Cfg::GetFreeRandomCfg, and Cfg::isCollision
   */
  CDInfo cdInfo;
  /**Will or won't generated Cfgs add to Roadmap graph.
   *If True, GenerateMapNodes::GenerateNodes
   *will add generated free Cfgs to roadmap graph.
   *Other generated Cfgs will be left nodes.
   */
  bool addNodes2Map;
  private:
    void ParseXML(TiXmlNode* in_pNode);
    void Reset();

};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class GenerateMapNodes declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
GenerateMapNodes<CFG>::
GenerateMapNodes() {

  BasicPRM<CFG>* basicPRM = new BasicPRM<CFG>();
  all.push_back(basicPRM);
  /*
  GaussPRM<CFG>* gaussPRM = new GaussPRM<CFG>();
  all.push_back(gaussPRM);

  BasicOBPRM<CFG>* basicOBPRM = new BasicOBPRM<CFG>();
  all.push_back(basicOBPRM);

  OBPRM<CFG>* obprm = new OBPRM<CFG>();
  all.push_back(obprm);
  
  BasicMAPRM<CFG>* basicMAPRM = new BasicMAPRM<CFG>();
  all.push_back(basicMAPRM);

  CSpaceMAPRM<CFG>* cspaceMAPRM = new CSpaceMAPRM<CFG>();
  all.push_back(cspaceMAPRM);
  
  OBMAPRM<CFG>* obmaprm = new OBMAPRM<CFG>();
  all.push_back(obmaprm);
  */
  addNodes2Map = true;
}

template <class CFG>
GenerateMapNodes<CFG>::
GenerateMapNodes(TiXmlNode* in_pNode) {
  LOG_MSG("GenerateMapNodes::GenearteMapNodes()",DEBUG_MSG);
  ParseXML(in_pNode);
  LOG_MSG("~GenerateMapNodes::GenearteMapNodes()",DEBUG_MSG);
}

template <class CFG>
void GenerateMapNodes<CFG>::
ParseXML(TiXmlNode* in_pNode) {
  LOG_MSG("GenerateMapNodes::ParseXML()",DEBUG_MSG);
  
  for( TiXmlNode* pChild = in_pNode->FirstChild(); 
      pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "BasicPRM") {
      BasicPRM<CFG>* basicPRM = new BasicPRM<CFG>(pChild);
      basicPRM->cdInfo = &cdInfo;
      selected.push_back(basicPRM);
    } else if(string(pChild->Value()) == "BasicOBPRM") {
      BasicOBPRM<CFG>* basicOBPRM = new BasicOBPRM<CFG>(pChild);
      basicOBPRM->cdInfo = &cdInfo;
      selected.push_back(basicOBPRM);
    }
  }
  /*
  GaussPRM<CFG>* gaussPRM = new GaussPRM<CFG>();
  all.push_back(gaussPRM);

  
  

  OBPRM<CFG>* obprm = new OBPRM<CFG>();
  all.push_back(obprm);
  
  BasicMAPRM<CFG>* basicMAPRM = new BasicMAPRM<CFG>();
  all.push_back(basicMAPRM);

  CSpaceMAPRM<CFG>* cspaceMAPRM = new CSpaceMAPRM<CFG>();
  all.push_back(cspaceMAPRM);
  
  OBMAPRM<CFG>* obmaprm = new OBMAPRM<CFG>();
  all.push_back(obmaprm);
  */
  addNodes2Map = true;
  
  /*
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
    for(int i=0; i<all.size(); ++i) {
      if(string(pChild->Value()) == all[i]->GetName()) {
        for( TiXmlNode* pChild2 = pChild->FirstChild(); pChild2 !=0; pChild2 = pChild2->NextSibling()) {
          if(string(pChild2->Value()) == "num_nodes") {
            all[i]->ParseXMLnum_nodes(pChild2);
          }
          all[i]->cdInfo = &cdInfo;
          selected.push_back(all[i]->CreateCopy());
          
        }
      }
    }
  }
  */
  if(selected.size() < 1)
    LOG_MSG("GenerateMapNodes::ParseXML() -- No methods selected",WARNING_MSG);
  
  LOG_MSG("~GenerateMapNodes::ParseXML()",DEBUG_MSG);
}


template <class CFG>
void GenerateMapNodes<CFG>::
Reset() {
  LOG_MSG("GenerateMapNodes::Reset()",DEBUG_MSG);
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
  
  all.clear();
  selected.clear();
  LOG_MSG("~GenerateMapNodes::Reset()",DEBUG_MSG);
}

template <class CFG>
GenerateMapNodes<CFG>::
~GenerateMapNodes() {
  LOG_MSG("GenerateMapNodes::~GenerateMapNodes()",DEBUG_MSG);
  Reset();
  LOG_MSG("~GenerateMapNodes::~GenerateMapNodes()",DEBUG_MSG);
}


template <class CFG>
vector<NodeGenerationMethod<CFG>*> 
GenerateMapNodes<CFG>::
GetDefault() {
  vector<NodeGenerationMethod<CFG>*> Default;
  BasicPRM<CFG>* basicPRM = new BasicPRM<CFG>();
  Default.push_back(basicPRM);
  return Default;
}


template <class CFG>
int 
GenerateMapNodes<CFG>::
ReadCommandLine(n_str_param* GNstrings[MAX_GN], int numGNs) {
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;

  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();
  
  typename vector<NodeGenerationMethod<CFG>*>::iterator itr;

  //go through the command line looking for method names
  for(int i=0; i<numGNs; i++) {

    std::istringstream _myistream(GNstrings[i]->GetValue());

    int argc = 0;
    char* argv[50];
    char cmdFields[50][100]; 
    while ( _myistream >> cmdFields[argc] ) {
      argv[argc] = (char*)(&cmdFields[argc]); 
      ++argc;
    }

    bool found = FALSE;
    try {
      int cmd_begin = 0;
      int cmd_argc = 0;
      char* cmd_argv[50];
      do {
	//go through the command line looking for method names
	for (itr = all.begin(); itr != all.end(); itr++) {
	  //If the method matches any of the supported methods ...
	  if ( !strcmp( argv[cmd_begin], (*itr)->GetName()) ) {
	    cmd_argc = 0;
	    bool is_method_name = false;
	    do {
	      cmd_argv[cmd_argc] = &(*(argv[cmd_begin+cmd_argc]));
	      cmd_argc++;
	      
	      typename vector<NodeGenerationMethod<CFG>*>::iterator itr_names;
	      is_method_name = false;
	      for (itr_names = all.begin(); itr_names != all.end() &&cmd_begin+cmd_argc < argc; itr_names++)
		if (!strcmp(argv[cmd_begin+cmd_argc],(*itr_names)->GetName())) {
		  is_method_name = true;
		  break;
		}
	    } while (! is_method_name && cmd_begin+cmd_argc < argc);	  

	    // .. use the parser of the matching method
	    (*itr)->ParseCommandLine(cmd_argc, cmd_argv);
	    // .., set their parameters
	    (*itr)->cdInfo = &cdInfo;
	    //  and push it back into the list of selected methods.	  
	    selected.push_back((*itr)->CreateCopy());	 
	    (*itr)->SetDefault();
	    found = TRUE;
	    break;
	  } 
	}
	if(!found)
	  break;
	cmd_begin = cmd_begin + cmd_argc;
      } while (cmd_begin < argc);
      if (!found)
	throw BadUsage();
    } catch (BadUsage) {
      cerr << "Command line error" << endl;
      PrintUsage(cerr);
      exit(-1); 
    }
  }

  //when there was no method selected, use the default
  if(selected.size() == 0) {
    selected = GenerateMapNodes<CFG>::GetDefault();
    for (itr = selected.begin(); itr != selected.end(); itr++) {
      (*itr)->cdInfo = &cdInfo;
    }
  }
  //cout << "selected:\n";
  //for(int j=0; j<selected.size(); j++)
  //  selected[j]->PrintValues(cout);

  return selected.size();
}


template <class CFG>
void 
GenerateMapNodes<CFG>::
PrintUsage(ostream& _os) {
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


template <class CFG>
void 
GenerateMapNodes<CFG>::
PrintValues(ostream& _os) {
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
void
GenerateMapNodes<CFG>::
PrintDefaults(ostream& _os) {
  vector<NodeGenerationMethod<CFG>*> Default;
  Default = GetDefault();
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
template <class WEIGHT>
void
GenerateMapNodes<CFG>::
GenerateNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	      CollisionDetection* cd, 
	      DistanceMetric* dm, vector<CFG>& nodes) {
cout << "GenerateMapNodes<CFG>::GenerateNodes" << endl;
  // clear generated nodes space
  nodes.erase(nodes.begin(),nodes.end());
  cout << "nodes.erase" << endl;
  typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
  cout << "I am here1" << endl << flush;
  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
    cout << "I am here2" << endl << flush;
#ifndef QUIET	
    Clock_Class clock;
    clock.StartClock((*itr)->GetName());
    cout<<"\n  "; clock.PrintName(); cout << " " << flush;
#endif	
    cout << "before (*itr)->GenerateNodes" << endl;
    (*itr)->GenerateNodes(_rm->GetEnvironment(), Stats, cd, dm, nodes);
    cout << "after (*itr)->GenerateNodes" << endl;
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec  \n" << flush;
#endif
  }

  //if that's what the user wants
  if (addNodes2Map) {
    _rm->m_pRoadmap->AddVertex(nodes); //add node to graph
  }
};




#endif


