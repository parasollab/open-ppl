#ifndef _ConnectMap_h_
#define _ConnectMap_h_


#include "Parameters.h"
#include "Roadmap.h"
#include "DistanceMetrics.h"
#include "Clock_Class.h"
#include "util.h"

#include <sstream>

//node connection methods
#include "NodeConnectionMethod.h"
#include "Closest.h"
#include "AllPairsNodeConnection.h"
//#include "UnconnectedClosest.h"
//#include "RandomConnect.h"
//#include "ModifiedLM.h"
//#include "ObstBased.h"
//#include "ClosestVE.h"
//#include "RRTexpand.h"
//#include "RayTracer.h" //??
#include "ConnectFirst.h"

//component connection methods
#include "ComponentConnectionMethod.h"
#include "ConnectCCs.h"
//#include "RRTcomponents.h"
//#include "RayTracer.h"
#include "Disconnect.h"

// MPRegion is used by region combination methods
///\todo Fix this include mess
///////////#include "MPRegion.h"
//class MPRegion;
#include "MPProblem.h"
#include "util.h"

// region connection methods
#include "RegionConnectionMethod.h"
#include "NaiveRegionConnect.h"
//#include "RegionOverlapMapCombine.h"


//#############################################################################
// A collection of component connection methods
template <class CFG, class WEIGHT>
class ConnectMap : public MPBaseObject{
 public:

  //////////////////////
  // Constructors and destructor
  ConnectMap();
  ConnectMap(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ConnectMap(Roadmap<CFG,WEIGHT>*, CollisionDetection*, 
	     DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~ConnectMap();

  //////////////////////
  // Access methods
  virtual vector<NodeConnectionMethod<CFG,WEIGHT>*> GetNodeDefault();
  virtual vector<ComponentConnectionMethod<CFG,WEIGHT>*> GetComponentDefault();
  virtual vector<RegionConnectionMethod<CFG,WEIGHT>*> GetRegionDefault();

  NodeConnectionMethod<CFG,WEIGHT>* GetNodeMethod(string& in_strLabel);
  ComponentConnectionMethod<CFG,WEIGHT>* GetComponentMethod(string& in_strLabel);
  RegionConnectionMethod<CFG,WEIGHT>* GetRegionMethod(string& in_strLabel);
  

  //////////////////////
  // I/O methods
  int ReadCommandLine(Input* input, Environment* env);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);
  void PrintOptions(ostream& out_os);

  //////////////////////
  // Core: Connection methods
  void Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	       CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges);

  void ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		    CollisionDetection* cd, DistanceMetric * dm,
		    LocalPlanners<CFG,WEIGHT>* lp,
		    bool addPartialEdge, bool addAllEdges);  
  void ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		    CollisionDetection* cd, DistanceMetric * dm,
		    LocalPlanners<CFG,WEIGHT>* lp,
		    bool addPartialEdge, bool addAllEdges,
		    vector<VID>& cfgs1, vector<VID>& cfgs2);  
  void ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		    CollisionDetection* cd, DistanceMetric * dm,
		    LocalPlanners<CFG,WEIGHT>* lp,
		    bool addPartialEdge, bool addAllEdges,
		    vector<vector<VID> >& cfgs);
  
  void ConnectComponents(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
			 CollisionDetection* cd, DistanceMetric* dm,
			 LocalPlanners<CFG,WEIGHT>* lp,
			 bool addPartialEdge, bool addAllEdges);
  void ConnectComponents(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
			 CollisionDetection* cd, DistanceMetric* dm,
			 LocalPlanners<CFG,WEIGHT>* lp,
			 bool addPartialEdge, bool addAllEdges,
			 vector<VID>& vids1, vector<VID>& vids2);
  void ConnectComponents(Roadmap<CFG,WEIGHT>* rm, Stat_Class& Stats,
			 CollisionDetection* cd, DistanceMetric* dm,
			 LocalPlanners<CFG,WEIGHT>* lp,
			 bool addPartialEdge, bool addAllEdges,
			 vector<vector<VID> >& vids);

  void ConnectRegions(CollisionDetection* cd, DistanceMetric* dm,
		      LocalPlanners<CFG,WEIGHT>* lp,
		      bool addPartialEdge, bool addAllEdges,
		      vector<MPRegion<CFG,WEIGHT>* > source_regions,
		      MPRegion<CFG,WEIGHT>* target_region=NULL);

 protected:
  //////////////////////
  // Data
  vector<NodeConnectionMethod<CFG,WEIGHT> *> all_node_methods;
  vector<NodeConnectionMethod<CFG,WEIGHT> *> selected_node_methods;

  vector<ComponentConnectionMethod<CFG,WEIGHT> *> all_component_methods;
  vector<ComponentConnectionMethod<CFG,WEIGHT> *> selected_component_methods;

  vector<RegionConnectionMethod<CFG,WEIGHT> *> all_region_methods;
  vector<RegionConnectionMethod<CFG,WEIGHT> *> selected_region_methods;
  
  n_str_param options; //component connection options

  void ParseXML(TiXmlNode* in_pNode);
  
 public:
  CDInfo cdInfo;
  static double connectionPosRes, ///< Position resolution for node connection
         connectionOriRes; ///< Orientation resolution for node connection
};

template <class CFG, class WEIGHT>
double ConnectMap<CFG, WEIGHT>::connectionPosRes = 0.05;

template <class CFG, class WEIGHT>
double ConnectMap<CFG, WEIGHT>::connectionOriRes = 0.05;


////////////////////////////////////////////////////////////////////////////
// ConnectMap: Methods
template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap() {
  //setup node connection methods
  selected_node_methods.clear();
  all_node_methods.clear();

  Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>();
  all_node_methods.push_back(closest);

  //UnconnectedClosest<CFG,WEIGHT>* unconnectedclosest = new UnconnectedClosest<CFG,WEIGHT>();
  //all_node_methods.push_back(unconnectedclosest);

  //RandomConnect<CFG,WEIGHT>* random = new RandomConnect<CFG,WEIGHT>();
  //all_node_methods.push_back(random);

  //ModifiedLM<CFG,WEIGHT>* lm = new ModifiedLM<CFG,WEIGHT>();
  //all_node_methods.push_back(lm);

  //ObstBased<CFG,WEIGHT>* obstbased = new ObstBased<CFG,WEIGHT>();
  //all_node_methods.push_back(obstbased);

  //ClosestVE<CFG,WEIGHT>* closestve = new ClosestVE<CFG,WEIGHT>();
  //all_node_methods.push_back(closestve);

  ConnectFirst<CFG,WEIGHT>* connectFirst = new ConnectFirst<CFG,WEIGHT>();
  all_node_methods.push_back(connectFirst);

  Disconnect<CFG,WEIGHT>* disconnect = new Disconnect<CFG,WEIGHT>();
  all_node_methods.push_back(disconnect);

  //setup component connection methods
  selected_component_methods.clear();
  all_component_methods.clear();

  ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>();
  all_component_methods.push_back(connectccs);

/*   RRTexpand<CFG,WEIGHT>* rrtexpand = new RRTexpand<CFG,WEIGHT>(); */
/*   all_component_methods.push_back(rrtexpand); */

/*   RRTcomponents<CFG,WEIGHT>* rrtcomp = new RRTcomponents<CFG,WEIGHT>(); */
/*   all_component_methods.push_back(rrtcomp); */

  //RayTracer<CFG,WEIGHT>* rt = new RayTracer<CFG,WEIGHT>();
  //all_component_methods.push_back(rt);


  //setup roadmap connection methods
  selected_region_methods.clear();
  all_region_methods.clear();

  NaiveRegionConnect<CFG,WEIGHT>* nmc = new NaiveRegionConnect<CFG,WEIGHT>();
  all_region_methods.push_back(nmc);

/*   OverlapRegionConnect<CFG,WEIGHT>* romc = new OverlapRegionConnect<CFG,WEIGHT>(); */
/*   all_region_methods.push_back(romc); */

  //Command-line-option string
  options.PutDesc("STRING",
   "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
   "\n\t\t\t\tRayTracer \tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
   "\n\t\t\t  RRTcomponents  INT INT INT INT INT (iter:10 factor:3 cc:3 o_clr:2 node_clr:4)" );
}

template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap(TiXmlNode* in_pNode, MPProblem* in_pProblem) : 
  MPBaseObject(in_pNode, in_pProblem){
  LOG_DEBUG_MSG("ConnectMap::ConnectMap()");
  ParseXML(in_pNode);
  
  
  if(selected_node_methods.size() < 1)
    LOG_WARNING_MSG("No Connection Methods selected!");

  
  


  LOG_DEBUG_MSG("~ConnectMap::ConnectMap()");
}


template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("ConnectMap::ParseXML()");
  
  connectionPosRes = GetMPProblem()->GetEnvironment()->GetPositionRes();
  cout << "connectionPosRes = " << connectionPosRes << endl;
  connectionOriRes = GetMPProblem()->GetEnvironment()->GetOrientationRes();     
  cout << "connectionOriRes = " << connectionOriRes << endl;

  for( TiXmlNode* pChild = in_pNode->FirstChild(); 
      pChild !=0; pChild = pChild->NextSibling()) {
    if(string(pChild->Value()) == "Closest") {
      cout << "ConnectMap found Closest" << endl;
      Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>(pChild,GetMPProblem());
      closest->cdInfo = &cdInfo;
      closest->connectionPosRes = connectionPosRes;
      closest->connectionOriRes = connectionOriRes; 
      all_node_methods.push_back(closest);
      selected_node_methods.push_back(closest);
    } else if(string(pChild->Value()) == "ConnectCCs") {
      cout << "ConnectMap found ConnectCCs" << endl;
      ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>(pChild,GetMPProblem());
      connectccs->cdInfo = &cdInfo;
      connectccs->connectionPosRes = connectionPosRes;
      connectccs->connectionOriRes = connectionOriRes; 
      all_component_methods.push_back(connectccs);
      selected_component_methods.push_back(connectccs);
    } else if(string(pChild->Value()) == "AllPairs") {
      cout << "ConnectMap found AllPairs" << endl;
      AllPairsNodeConnection<CFG,WEIGHT>* allPairs = 
          new AllPairsNodeConnection<CFG,WEIGHT>(pChild,GetMPProblem());
      allPairs->cdInfo = &cdInfo;
      allPairs->connectionPosRes = connectionPosRes;
      allPairs->connectionOriRes = connectionOriRes; 
      all_node_methods.push_back(allPairs);
      selected_node_methods.push_back(allPairs);
    } else if(string(pChild->Value()) == "Disconnect") {
      cout << "ConnectMap found Disconnect" << endl;
      Disconnect<CFG,WEIGHT>* disconnect = 
          new Disconnect< CFG,WEIGHT >(pChild,GetMPProblem());
      disconnect->cdInfo = &cdInfo;
      disconnect->connectionPosRes = connectionPosRes;
      disconnect->connectionOriRes = connectionOriRes; 
      all_node_methods.push_back(disconnect);
      selected_node_methods.push_back(disconnect);
    }
  }
  
/*
  //setup node connection methods
  selected_node_methods.clear();
  all_node_methods.clear();

  Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>();
  all_node_methods.push_back(closest);

  //UnconnectedClosest<CFG,WEIGHT>* unconnectedclosest = new UnconnectedClosest<CFG,WEIGHT>();
  //all_node_methods.push_back(unconnectedclosest);

  RandomConnect<CFG,WEIGHT>* random = new RandomConnect<CFG,WEIGHT>();
  all_node_methods.push_back(random);

  //ModifiedLM<CFG,WEIGHT>* lm = new ModifiedLM<CFG,WEIGHT>();
  //all_node_methods.push_back(lm);

  ObstBased<CFG,WEIGHT>* obstbased = new ObstBased<CFG,WEIGHT>();
  all_node_methods.push_back(obstbased);

  ClosestVE<CFG,WEIGHT>* closestve = new ClosestVE<CFG,WEIGHT>();
  all_node_methods.push_back(closestve);

  ConnectFirst<CFG,WEIGHT>* connectFirst = new ConnectFirst<CFG,WEIGHT>();
  all_node_methods.push_back(connectFirst);


  //setup component connection methods
  selected_component_methods.clear();
  all_component_methods.clear();

  ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>();
  all_component_methods.push_back(connectccs);

  ///\todo Fix closest .... for some reason doesnt match them up.
  for( TiXmlNode* pChild = in_pNode->FirstChild(); pChild !=0; pChild = pChild->NextSibling()) {
  for(int i=0; i<all_component_methods.size(); ++i) {
  if(string(pChild->Value()) == all_component_methods[i]->GetName()) {
  cout << "ConnectionMethod selected = " << all_component_methods[i]->GetName() << endl;
  selected_component_methods.push_back(all_component_methods[i]);
}
}
}
  
  */
    
  LOG_DEBUG_MSG("~ConnectMap::ParseXML()");
}

template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
ConnectMap(Roadmap<CFG,WEIGHT> * rdmp, CollisionDetection* cd, 
	   DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp) : 
  options("-cComponents") {
  ConnectMap();
}


template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::
~ConnectMap() {
  selected_node_methods.clear();
  all_node_methods.clear();

  selected_component_methods.clear();
  all_component_methods.clear();

  selected_region_methods.clear();
  all_region_methods.clear();
}

template <class CFG, class WEIGHT>
NodeConnectionMethod<CFG,WEIGHT>* 
ConnectMap<CFG,WEIGHT>::
GetNodeMethod(string& in_strLabel) {

  typename vector<NodeConnectionMethod<CFG, WEIGHT>*>::iterator I;
  for(I = selected_node_methods.begin(); I != selected_node_methods.end(); ++I) {
    if((*I)->GetLabel() == in_strLabel) {
      return (*I);
    }
  }
  LOG_ERROR_MSG("ConnectMap:: cannot find NodeConnectionMethod label = " << in_strLabel);
}
  
  
template <class CFG, class WEIGHT>
ComponentConnectionMethod<CFG,WEIGHT>* 
ConnectMap<CFG,WEIGHT>::
GetComponentMethod(string& in_strLabel) {

  typename vector<ComponentConnectionMethod<CFG, WEIGHT>*>::iterator I;
  for(I = selected_component_methods.begin(); 
    I != selected_component_methods.end(); ++I) {
      if((*I)->GetLabel() == in_strLabel) {
      return (*I);
    }
  }
  LOG_ERROR_MSG("ConnectMap:: cannot find ComponentConnectionMethod label = " << in_strLabel);
}  


template <class CFG, class WEIGHT>
RegionConnectionMethod<CFG,WEIGHT>* 
ConnectMap<CFG,WEIGHT>::
GetRegionMethod(string& in_strLabel) {

  typename vector<RegionConnectionMethod<CFG, WEIGHT>*>::iterator I;
  for(I = selected_region_methods.begin(); 
    I != selected_region_methods.end(); ++I) {
    if(*I->GetLabel() == in_strLabel) {
      return &(*I);
    }
  }
  LOG_ERROR_MSG("ConnectMap:: cannot find RegionConnectionMethod label = " << in_strLabel);
  exit(-1);
}  


template <class CFG, class WEIGHT>
vector<NodeConnectionMethod<CFG,WEIGHT> *> 
ConnectMap<CFG,WEIGHT>::
GetNodeDefault() {
  vector<NodeConnectionMethod<CFG,WEIGHT> *> tmp;
  Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>();
  closest->SetDefault();
  tmp.push_back(closest);    
  
  return tmp;
}


template <class CFG, class WEIGHT>
vector<ComponentConnectionMethod<CFG,WEIGHT> *> 
ConnectMap<CFG,WEIGHT>::
GetComponentDefault() {
  vector<ComponentConnectionMethod<CFG,WEIGHT> *> tmp;

  return tmp;
}


template <class CFG, class WEIGHT>
vector<RegionConnectionMethod<CFG,WEIGHT> *> 
ConnectMap<CFG,WEIGHT>::
GetRegionDefault() {
  vector<RegionConnectionMethod<CFG,WEIGHT> *> tmp;
  
  return tmp;
}



template <class CFG, class WEIGHT>
int 
ConnectMap<CFG,WEIGHT>::
ReadCommandLine(Input* input, Environment* env) {
  connectionPosRes = env->GetPositionRes();
  connectionOriRes = env->GetOrientationRes();   
  
  typename vector<NodeConnectionMethod<CFG, WEIGHT>*>::iterator I;
  for(I = selected_node_methods.begin(); 
      I != selected_node_methods.end(); ++I)
    delete *I;
  selected_node_methods.clear();

  typename vector<ComponentConnectionMethod<CFG, WEIGHT>*>::iterator J;
  for(J = selected_component_methods.begin(); 
      J != selected_component_methods.end(); ++J)
    delete *J;
  selected_component_methods.clear();  


  typename vector<RegionConnectionMethod<CFG, WEIGHT>*>::iterator K;
  for(K = selected_region_methods.begin(); 
      K != selected_region_methods.end(); ++K)
    delete *K;
  selected_region_methods.clear();

  //go through the command line looking for method names
  for(int i=0; i<input->numCNs; ++i) {
    std::istringstream _myistream(input->CNstrings[i]->GetValue());
    char cnname[100];

    while (_myistream >> cnname) {
      bool found = FALSE;
      try {
        //go through the command line looking for method names
        for(I = all_node_methods.begin(); 
	    I != all_node_methods.end(); ++I) {
          //If the method matches any of the supported methods ...
	  if(!strcmp(cnname, (*I)->GetName())) {
  	    // .. use the parser of the matching method
	    (*I)->ParseCommandLine(_myistream);
	    // .., set their parameters
	    (*I)->cdInfo = &cdInfo;
	    (*I)->connectionPosRes = connectionPosRes;
	    (*I)->connectionOriRes = connectionOriRes; 
	    //  and push it back into the list of selected methods.	  
	    selected_node_methods.push_back((*I)->CreateCopy());	 
	    (*I)->SetDefault();
	    found = TRUE;
	    break;
	  } 
        }

        if(!found) {
	  //go through the command line looking for method names
	  for(J = all_component_methods.begin(); 
	      J != all_component_methods.end(); ++J) {
	    //If the method matches any of the supported methods ...
	    if(!strcmp(cnname, (*J)->GetName())) {
	      // .. use the parser of the matching method
	      (*J)->ParseCommandLine(_myistream);
	      // .., set their parameters
	      (*J)->cdInfo = &cdInfo;
	      (*J)->connectionPosRes = connectionPosRes;
	      (*J)->connectionOriRes = connectionOriRes; 
	      //  and push it back into the list of selected methods.	  
	      selected_component_methods.push_back((*J)->CreateCopy());	 
	      (*J)->SetDefault();
	      found = TRUE;
	      break;
	    } 
	  }
	}
	
	if(!found) {
	  //go through the command line looking for method names
	  for(K = all_region_methods.begin(); 
	      K != all_region_methods.end(); ++K) {
	    //If the method matches any of the supported methods ...
	    if(!strcmp(cnname, (*K)->GetName())) {
	      // .. use the parser of the matching method
	      (*K)->ParseCommandLine(_myistream);
	      // .., set their parameters
	      (*K)->cdInfo = &cdInfo;
	      (*K)->connectionPosRes = connectionPosRes;
	      (*K)->connectionOriRes = connectionOriRes; 	      
	      //  and push it back into the list of selected methods.	  
	      selected_region_methods.push_back((*K)->CreateCopy());	 
	      (*K)->SetDefault();
	      found = TRUE;
	      break;
	    } 
	  }
	}

	if(!found)
  	  throw BadUsage();
	
      } catch (BadUsage) {
        cerr << "Command line error"<< endl;
        PrintUsage(cerr);
        exit(-1); 
      }
    }   
  }

  //when there was no node method selected, use the default
  if(selected_node_methods.empty()) {
    selected_node_methods = this->GetNodeDefault();
    for(I = selected_node_methods.begin(); 
	I != selected_node_methods.end(); ++I) {
      (*I)->cdInfo = &cdInfo;
      (*I)->connectionPosRes= connectionPosRes;
      (*I)->connectionOriRes= connectionOriRes; 
    }
  }

  //when there was no component method selected, use the default
  if(selected_component_methods.empty()) {
    selected_component_methods = this->GetComponentDefault();
    for(J = selected_component_methods.begin(); 
	J != selected_component_methods.end(); ++J) {
      (*J)->cdInfo = &cdInfo;
      (*J)->connectionPosRes= connectionPosRes;
      (*J)->connectionOriRes= connectionOriRes; 
    }
  }

  //when there was no region method selected, use the default
  if(selected_region_methods.empty()) {
    selected_region_methods = this->GetRegionDefault();
    for(K = selected_region_methods.begin(); 
	K != selected_region_methods.end(); ++K) {
      (*K)->cdInfo = &cdInfo;
      (*K)->connectionPosRes= connectionPosRes;
      (*K)->connectionOriRes= connectionOriRes; 
    }
  }

  return selected_node_methods.size() + selected_component_methods.size() + selected_region_methods.size();
}

template <class CFG, class WEIGHT>
void
ConnectMap<CFG,WEIGHT>::
PrintOptions(ostream& out_os) {
  out_os << "  Connection Methods" << endl;
  typename vector<NodeConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I = all_node_methods.begin(); I != all_node_methods.end(); ++I)
    (*I)->PrintOptions(out_os);

  typename vector<ComponentConnectionMethod<CFG,WEIGHT>*>::iterator J;
  for(J = all_component_methods.begin(); J != all_component_methods.end(); ++J)
    (*J)->PrintOptions(out_os);

  typename vector<RegionConnectionMethod<CFG,WEIGHT>*>::iterator K;
  for(K = all_region_methods.begin(); K != all_region_methods.end(); ++K)
    (*K)->PrintOptions(out_os);
  
}


template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
  typename vector<NodeConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I = all_node_methods.begin(); I != all_node_methods.end(); ++I)
    (*I)->PrintUsage(_os);

  typename vector<ComponentConnectionMethod<CFG,WEIGHT>*>::iterator J;
  for(J = all_component_methods.begin(); J != all_component_methods.end(); ++J)
    (*J)->PrintUsage(_os);

  typename vector<RegionConnectionMethod<CFG,WEIGHT>*>::iterator K;
  for(K = all_region_methods.begin(); K != all_region_methods.end(); ++K)
    (*K)->PrintUsage(_os);

}


template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
PrintValues(ostream& _os){
  typename vector<NodeConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I = selected_node_methods.begin(); 
      I != selected_node_methods.end(); ++I)
    (*I)->PrintValues(_os);

  typename vector<ComponentConnectionMethod<CFG,WEIGHT>*>::iterator J;
  for(J = selected_component_methods.begin(); 
      J != selected_component_methods.end(); ++J)
    (*J)->PrintValues(_os);

  typename vector<RegionConnectionMethod<CFG,WEIGHT>*>::iterator K;
  for(K = selected_region_methods.begin(); 
      K != selected_region_methods.end(); ++K)
    (*K)->PrintValues(_os);

};


template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
PrintDefaults(ostream& _os) {
  vector<NodeConnectionMethod<CFG,WEIGHT>*> Default;
  Default = this->GetNodeDefault();
  typename vector<NodeConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I = Default.begin(); I != Default.end(); ++I)
    (*I)->PrintValues(_os);

  vector<ComponentConnectionMethod<CFG,WEIGHT>*> Default2;
  Default2 = this->GetComponentDefault();
  typename vector<ComponentConnectionMethod<CFG,WEIGHT>*>::iterator J;
  for(J = Default2.begin(); J != Default2.end(); ++J)
    (*J)->PrintValues(_os);

  vector<RegionConnectionMethod<CFG,WEIGHT>*> Default3;
  Default3 = this->GetRegionDefault();
  typename vector<RegionConnectionMethod<CFG,WEIGHT>*>::iterator K;
  for(K = Default3.begin(); K != Default3.end(); ++K)
    (*K)->PrintValues(_os);

}


template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
Connect(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	CollisionDetection* cd, DistanceMetric* dm,
	LocalPlanners<CFG,WEIGHT>* lp,
	bool addPartialEdge, bool addAllEdges) {
  ConnectNodes(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges);
  ConnectComponents(_rm, Stats, cd, dm, lp, addPartialEdge, addAllEdges);
}

template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	     CollisionDetection* cd, DistanceMetric * dm,
	     LocalPlanners<CFG,WEIGHT>* lp,
	     bool addPartialEdge, bool addAllEdges) {
  typename vector<NodeConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_node_methods.begin(); 
      itr != selected_node_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}
 

template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	     CollisionDetection* cd, DistanceMetric * dm,
	     LocalPlanners<CFG,WEIGHT>* lp,
	     bool addPartialEdge, bool addAllEdges,
	     vector<VID>& cfgs1, vector<VID>& cfgs2) {
  typename vector<NodeConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_node_methods.begin(); 
      itr != selected_node_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,cfgs1,cfgs2);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}


template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectNodes(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
	     CollisionDetection* cd, DistanceMetric * dm,
	     LocalPlanners<CFG,WEIGHT>* lp,
	     bool addPartialEdge, bool addAllEdges,
	     vector<vector<VID> >& cfgs) {
  typename vector<NodeConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_node_methods.begin(); 
      itr != selected_node_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,cfgs);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}

  
template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd, DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, bool addAllEdges) {
  typename vector<ComponentConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_component_methods.begin(); 
      itr != selected_component_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}
 

template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd, DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, bool addAllEdges,
		  vector<VID>& vids1, vector<VID>& vids2) {
  typename vector<ComponentConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_component_methods.begin(); 
      itr != selected_component_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,vids1,vids2);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}

template <class CFG, class WEIGHT>
void 
ConnectMap<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, Stat_Class& Stats,
		  CollisionDetection* cd, DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge, bool addAllEdges,
		  vector<vector<VID> >& vids) {
  typename vector<ComponentConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for(itr = selected_component_methods.begin(); 
      itr != selected_component_methods.end(); itr++) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->Connect(_rm,Stats,cd,dm,lp,addPartialEdge,addAllEdges,vids);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}

template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::
ConnectRegions(CollisionDetection* cd, DistanceMetric* dm,
	       LocalPlanners<CFG,WEIGHT>* lp,
	       bool addPartialEdge, bool addAllEdges,
	       vector<MPRegion<CFG,WEIGHT>* > source_regions,
	       MPRegion<CFG,WEIGHT>* target_region) {
  typename vector<RegionConnectionMethod<CFG,WEIGHT>*>::iterator itr;
  for (itr = selected_region_methods.begin(); itr != selected_region_methods.end(); itr++ ) {
#ifndef QUIET
    Clock_Class clock;
    clock.StartClock((*itr)->GetName());
    cout<<"\n  "; 
    clock.PrintName(); 
    cout << " " << flush;
#endif
    
    (*itr)->Connect(target_region, source_regions, *this, *cd, *dm, *lp,
		    addPartialEdge, addAllEdges);
#ifndef QUIET
    clock.StopClock();
    cout << clock.GetClock_SEC() << " sec  \n" << flush;
#endif
  }
}


#endif /*_ConnectMap_h_*/
