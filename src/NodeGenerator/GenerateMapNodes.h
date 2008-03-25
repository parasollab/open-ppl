
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
#include "GMSPolyhedron.h"

//Include node generation methods
#include "PRMUniform.h"
#include "PRMGauss.h"
#include "MAPRMCSpace.h"
#include "MAPRMWSpace.h"
#include "BasicOBPRM.h"
#include "OBPRM.h"
#include "OBMAPRM.h"
#include "PRMBridgeTest.h"
#include "PRMModelBased.h"
#include "PRMActiveLearning.h"

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
  GenerateMapNodes(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.	
  virtual ~GenerateMapNodes();
  
  void PrintOptions(ostream& out_os);

  //@}

  //////////////////////
  // Access methods
  static vector<NodeGenerationMethod<CFG>*> GetDefault();

  //////////////////////
  // I/O methods
  void PrintDefaults(ostream& _os);
  NodeGenerationMethod<CFG>* GetMethod(string& in_strLabel);
  

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
  private:
    void ParseXML(XMLNodeReader& in_Node);
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

  GaussPRM<CFG>* gaussPRM = new GaussPRM<CFG>();
  all.push_back(gaussPRM);

  BasicOBPRM<CFG>* basicOBPRM = new BasicOBPRM<CFG>();
  all.push_back(basicOBPRM);

  OBPRM<CFG>* obprm = new OBPRM<CFG>();
  all.push_back(obprm);
  /*
  BasicMAPRM<CFG>* basicMAPRM = new BasicMAPRM<CFG>();
  all.push_back(basicMAPRM);
  */
  CSpaceMAPRM<CFG>* cspaceMAPRM = new CSpaceMAPRM<CFG>();
  all.push_back(cspaceMAPRM);
  /*
  OBMAPRM<CFG>* obmaprm = new OBMAPRM<CFG>();
  all.push_back(obmaprm);
  */

  BridgeTestPRM<CFG>* bridgePRM = new BridgeTestPRM<CFG>();
  all.push_back(bridgePRM);

  PRMModelBased<CFG>* modelBasedPRM = new PRMModelBased<CFG>();
  all.push_back(modelBasedPRM);
  
  PRMActiveLearning<CFG>* activeLearningPRM = new PRMActiveLearning<CFG>();
  all.push_back(activeLearningPRM);
  
  addNodes2Map = true;
}

template <class CFG>
GenerateMapNodes<CFG>::
GenerateMapNodes(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
     MPBaseObject(in_Node,in_pProblem) {
  
  LOG_DEBUG_MSG("GenerateMapNodes::GenearteMapNodes()");
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~GenerateMapNodes::GenearteMapNodes()");
}

template <class CFG>
void GenerateMapNodes<CFG>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GenerateMapNodes::ParseXML()");

  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "BasicPRM") {
      BasicPRM<CFG>* basicPRM = new BasicPRM<CFG>(*citr, GetMPProblem());
      basicPRM->cdInfo = &cdInfo;
      selected.push_back(basicPRM);
    } else if(citr->getName() == "BasicOBPRM") {
      BasicOBPRM<CFG>* basicOBPRM = new BasicOBPRM<CFG>(*citr, GetMPProblem());
      basicOBPRM->cdInfo = &cdInfo;
      selected.push_back(basicOBPRM);
    } else if(citr->getName() == "BridgeTestPRM") {
      BridgeTestPRM<CFG>* bridgeTest = new BridgeTestPRM<CFG>(*citr, GetMPProblem());
      bridgeTest->cdInfo = &cdInfo;
      selected.push_back(bridgeTest);
    } else if(citr->getName() == "GaussPRM") {
      GaussPRM<CFG>* gaussPRM = new GaussPRM<CFG>(*citr, GetMPProblem());
      gaussPRM->cdInfo = &cdInfo;
      selected.push_back(gaussPRM);
    } else if(citr->getName() == "CSpaceMAPRM") {
      CSpaceMAPRM<CFG>* cspaceMAPRM = new CSpaceMAPRM<CFG>(*citr, GetMPProblem());
      cspaceMAPRM->cdInfo = &cdInfo;
      selected.push_back(cspaceMAPRM);
    } else if(citr->getName() == "OBPRM") {
      OBPRM<CFG>* obprm = new OBPRM<CFG>(*citr,GetMPProblem());
      //all.push_back(obprm);
      obprm->cdInfo = &cdInfo;
      selected.push_back(obprm);
    } else {
      citr->warnUnknownNode();
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

  BridgeTestPRM<CFG>* bridgePRM = new BridgeTestPRM<CFG>();
  all.push_back(bridgePRM);


  */
  addNodes2Map = true;
  
  
  if(selected.size() < 1) {
    LOG_WARNING_MSG("GenerateMapNodes::ParseXML() -- No methods selected");
  }
  
  LOG_DEBUG_MSG("~GenerateMapNodes::ParseXML()");
}

template <class CFG>
NodeGenerationMethod<CFG>* 
GenerateMapNodes<CFG>::GetMethod(string& in_strLabel) {
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++) {
    if((*I)->GetLabel() == in_strLabel) {
      return *I;
    }
  }
  LOG_ERROR_MSG("GenerateMapNodes:: cannot find NodeGenerationMethod label = " << in_strLabel);
  exit(-1);
}

template <class CFG>
void GenerateMapNodes<CFG>::
Reset() {
  LOG_DEBUG_MSG("GenerateMapNodes::Reset()");
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;

  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
  
  all.clear();
  selected.clear();
  LOG_DEBUG_MSG("~GenerateMapNodes::Reset()");
}

template <class CFG>
GenerateMapNodes<CFG>::
~GenerateMapNodes() {
  LOG_DEBUG_MSG("GenerateMapNodes::~GenerateMapNodes()");
  Reset();
  LOG_DEBUG_MSG("~GenerateMapNodes::~GenerateMapNodes()");
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
void
GenerateMapNodes<CFG>::
PrintDefaults(ostream& _os) {
  vector<NodeGenerationMethod<CFG>*> Default;
  Default = GetDefault();
  typename vector<NodeGenerationMethod<CFG>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintOptions(_os);
}


template <class CFG>
template <class WEIGHT>
void
GenerateMapNodes<CFG>::
GenerateNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	      CollisionDetection* cd, 
	      DistanceMetric* dm, vector<CFG>& nodes) {

  LOG_DEBUG_MSG("GenerateMapNodes::GenerateNodes");
  // clear generated nodes space
  nodes.erase(nodes.begin(),nodes.end());
  typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
#ifndef QUIET	
    Clock_Class clock;
    clock.StartClock((*itr)->GetName());
    cout<<"\n  "; clock.PrintName(); cout << " " << flush;
#endif	
    (*itr)->GenerateNodes(_rm->GetEnvironment(), Stats, cd, dm, nodes);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec  \n" << flush;
#endif
    if (addNodes2Map) {
      _rm->m_pRoadmap->AddVertex(nodes); //add node to graph
    }
    nodes.erase(nodes.begin(),nodes.end());
  }

  //if that's what the user wants
  /////Moved this section up to for loop.
  //if (addNodes2Map) {
  //  _rm->m_pRoadmap->AddVertex(nodes); //add node to graph
  //}
  
  LOG_DEBUG_MSG("~GenerateMapNodes::GenerateNodes");
};


template <class CFG>
void
GenerateMapNodes<CFG>::
PrintOptions(ostream& out_os) {
  out_os << "  Node Generation Mehods" << endl;
  typename vector<NodeGenerationMethod<CFG>*>::iterator itr;
  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
    (*itr)->PrintOptions(out_os);
  }
}


#endif


