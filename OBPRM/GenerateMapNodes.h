// $Id$

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

#ifdef __K2
  #include <strstream.h>
#else
  #include <strstream>
#endif

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
class GenerateMapNodes {
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
  void GenerateNodes(Roadmap<CFG, WEIGHT>* _rm, CollisionDetection* cd, 
		     DistanceMetric* dm, vector<CFG>& nodes);

 private:
  //////////////////////
  // Data
  vector<NodeGenerationMethod<CFG>*> all;
  vector<NodeGenerationMethod<CFG>*> selected;

 public:
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
  
  addNodes2Map = true;
}


template <class CFG>
GenerateMapNodes<CFG>::
~GenerateMapNodes() {
  vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
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
  vector<NodeGenerationMethod<CFG>*>::iterator I;

  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();
  
  vector<NodeGenerationMethod<CFG>*>::iterator itr;

  //go through the command line looking for method names
  for(int i=0; i<numGNs; i++) {

    istrstream _myistream(GNstrings[i]->GetValue());

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
	      
	      vector<NodeGenerationMethod<CFG>*>::iterator itr_names;
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
}


template <class CFG>
void 
GenerateMapNodes<CFG>::
PrintUsage(ostream& _os) {
  vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}


template <class CFG>
void 
GenerateMapNodes<CFG>::
PrintValues(ostream& _os) {
  vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
void
GenerateMapNodes<CFG>::
PrintDefaults(ostream& _os) {
  vector<NodeGenerationMethod<CFG>*> Default;
  Default = GetDefault();
  vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG>
template <class WEIGHT>
void
GenerateMapNodes<CFG>::
GenerateNodes(Roadmap<CFG, WEIGHT>* _rm, CollisionDetection* cd, 
	      DistanceMetric* dm, vector<CFG>& nodes) {
  // clear generated nodes space
  nodes.erase(nodes.begin(),nodes.end());

  vector<NodeGenerationMethod<CFG>*>::iterator itr;
  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
#ifndef QUIET	
    Clock_Class clock;
    clock.StartClock((*itr)->GetName());
    cout<<"\n  "; clock.PrintName(); cout << " " << flush;
#endif	
    
    (*itr)->GenerateNodes(_rm->GetEnvironment(), cd, dm, nodes);
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


