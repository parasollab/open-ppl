#ifndef _ConnectMap_h_
#define _ConnectMap_h_

#include "Parameters.h"
#include "Roadmap.h"
#include "DistanceMetrics.h"
#include "Clock_Class.h"
#include "util.h"
#ifdef __K2
  #include <strstream.h>
#else
  #include <strstream>
#endif

#include "RayTracer.h"
#include "RRTexpand.h"
#include "RRTcomponents.h"
#include "ConnectCCs.h"
#include "Closest.h"
#include "ClosestVE.h"
#include "RandomConnect.h"
#include "ObstBased.h"
#include "ModifiedLM.h"

//#define MAX_CN 10
const double MAX_DIST =  1e10;
//#############################################################################
// A collection of component connection methods
template <class CFG, class WEIGHT>
class ConnectMap {
 public:

  //////////////////////
  // Constructors and destructor
  ConnectMap();
  ConnectMap(Roadmap<CFG,WEIGHT>*, CollisionDetection*, 
		       DistanceMetric*, LocalPlanners<CFG,WEIGHT>*);
  ~ConnectMap();

  //////////////////////
  // Access methods
  static vector<ConnectionMethod<CFG,WEIGHT> *> GetDefault();

  //////////////////////
  // I/O methods
  int ReadCommandLine(Input* input, Environment* env);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void PrintDefaults(ostream& _os);

  //////////////////////
  // Core: Connection methods
  void ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, 
			 CollisionDetection* cd , 
			 DistanceMetric * dm,
			 LocalPlanners<CFG,WEIGHT>* lp,
			 bool addPartialEdge,
			 bool addAllEdges);

 private:

  //////////////////////
  // Data
  vector<ConnectionMethod<CFG,WEIGHT> *> all;
  vector<ConnectionMethod<CFG,WEIGHT> *> selected;
  
  n_str_param options; //component connection options
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
ConnectMap<CFG,WEIGHT>::ConnectMap() {
  selected.clear();
  all.clear();

  Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>();
  all.push_back(closest);

  RandomConnect<CFG,WEIGHT>* random = new RandomConnect<CFG,WEIGHT>();
  all.push_back(random);

  ObstBased<CFG,WEIGHT>* obstbased = new ObstBased<CFG,WEIGHT>();
  all.push_back(obstbased);

  ClosestVE<CFG,WEIGHT>* closestve = new ClosestVE<CFG,WEIGHT>();
  all.push_back(closestve);

  RRTexpand<CFG,WEIGHT>* rrtexpand = new RRTexpand<CFG,WEIGHT>();
  all.push_back(rrtexpand);

  RRTcomponents<CFG,WEIGHT>* rrtcomp = new RRTcomponents<CFG,WEIGHT>();
  all.push_back(rrtcomp);

  ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>();
  all.push_back(connectccs);
  
  RayTracer<CFG,WEIGHT>* rt = new RayTracer<CFG,WEIGHT>();
  all.push_back(rt);

  ModifiedLM<CFG,WEIGHT>* lm = new ModifiedLM<CFG,WEIGHT>();
  all.push_back(lm);

  //Command-line-option string
  options.PutDesc("STRING",
   "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
   "\n\t\t\t\tRayTracer \tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
   "\n\t\t\t  RRTcomponents  INT INT INT INT INT (iter:10 factor:3 cc:3 o_clr:2 node_clr:4)" );
}

template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::ConnectMap(Roadmap<CFG,WEIGHT> * rdmp, CollisionDetection* cd, DistanceMetric* dm, LocalPlanners<CFG,WEIGHT>* lp) : 
  options("-cComponents") {
  selected.clear();
  all.clear();
  //need to fill out the vector of connection_methods 
  
  //  RRT<CFG,WEIGHT>* rrt = new RRT<CFG,WEIGHT>(rdmp,cd,dm,lp);
  //  all.push_back(rrt);

  //  ConnectCCs<CFG,WEIGHT>* connectccs = new ConnectCCs<CFG,WEIGHT>(rdmp,cd,dm,lp);
  //  all.push_back(connectccs);

  //  RayTracer<CFG,WEIGHT>* rt = new RayTracer<CFG,WEIGHT>(rdmp,cd,dm,lp);
  //  all.push_back(rt);
  
  //Command-line-option string
  options.PutDesc("STRING",
   "\n\t\t\tPick any combo: default RayTracer targetOriented 1 10000 10000"
   "\n\t\t\t\tRayTracer \tSTRING \tINT \tINT \tINT \tSTRING \tINT \tINT (bouncingMode:targetOriented maxRays:1 maxBounces:10000 maxRayLength:10000 \tschedulingMode:largestToSmallest scheduleMaxSize:20 sampleMaxSize:10)"
   "\n\t\t\t  RRTcomponents  INT INT INT INT INT (iter:10 factor:3 cc:3 o_clr:2 node_clr:4)" );
}


template <class CFG, class WEIGHT>
ConnectMap<CFG,WEIGHT>::~ConnectMap() {
  selected.clear();
  all.clear();
}


template <class CFG, class WEIGHT>
vector<ConnectionMethod<CFG,WEIGHT> *> ConnectMap<CFG,WEIGHT>::GetDefault() {
    vector<ConnectionMethod<CFG,WEIGHT> *> tmp;
  Closest<CFG,WEIGHT>* closest = new Closest<CFG,WEIGHT>();
  closest->SetDefault();
  tmp.push_back(closest);    

    return tmp;
  }
// For compatability with connectmapnodes

template <class CFG, class WEIGHT>
int ConnectMap<CFG,WEIGHT>::ReadCommandLine(Input* input, Environment* env) {

  connectionPosRes = env->GetPositionRes();
  connectionOriRes = env->GetOrientationRes();   

  vector<ConnectionMethod<CFG, WEIGHT>*>::iterator I;
  
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  selected.clear();

  vector<ConnectionMethod<CFG,WEIGHT> *>::iterator itr, itr_names;

  //go through the command line looking for method names
  for(int i=0; i< input->numCNs; i++) {
    istrstream _myistream(input->CNstrings[i]->GetValue());
    char cnname[100];

    while (_myistream >> cnname) {
      bool found = FALSE;
      try {
        //go through the command line looking for method names
        for (itr = all.begin(); itr != all.end(); itr++) {
          //If the method matches any of the supported methods ...
	  if ( !strcmp(cnname, (*itr)->GetName()) ) {
  	    // .. use the parser of the matching method
	    (*itr)->ParseCommandLine(_myistream);
	    // .., set their parameters
	    (*itr)->cdInfo = &cdInfo;
	    (*itr)->connectionPosRes = connectionPosRes;
	    (*itr)->connectionOriRes = connectionOriRes; 

	    //  and push it back into the list of selected methods.	  
	    selected.push_back((*itr)->CreateCopy());	 
	    (*itr)->SetDefault();
	    found = TRUE;
	    break;
	  } 
        }
        if (!found)
  	  throw BadUsage();
      } catch (BadUsage) {
        cerr << "Command line error"<< endl;
        PrintUsage(cerr);
        exit(-1); 
      }
    }   
  }

  //when there was no method selected, use the default
  if(selected.size() == 0) {
    selected = ConnectMap<CFG,WEIGHT>::GetDefault();
    for (itr = selected.begin(); itr != selected.end(); itr++) {
      (*itr)->cdInfo = &cdInfo;
      (*itr)->connectionPosRes= connectionPosRes;
      (*itr)->connectionOriRes= connectionOriRes; 
    }
  }

}


template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::PrintUsage(ostream& _os) {
  vector<ConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);

}

template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::PrintValues(ostream& _os){
  vector<ConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
};

template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::PrintDefaults(ostream& _os) {
  vector<ConnectionMethod<CFG,WEIGHT>*> Default;
  Default = GetDefault();
  vector<ConnectionMethod<CFG,WEIGHT>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}

template <class CFG, class WEIGHT>
void ConnectMap<CFG,WEIGHT>::
ConnectComponents(Roadmap<CFG, WEIGHT>* _rm, 
		  CollisionDetection* cd , 
		  DistanceMetric * dm,
		  LocalPlanners<CFG,WEIGHT>* lp,
		  bool addPartialEdge,
		  bool addAllEdges) {
  vector<ConnectionMethod<CFG,WEIGHT> *>::iterator itr;
  for ( itr = selected.begin(); itr != selected.end(); itr++ ) {
#ifndef QUIET
    Clock_Class clock;
    char* name = " ";
    clock.StartClock(name);
    cout<<"\n ";
    //clock.PrintName();
    cout << flush;
#endif
    (*itr)->ConnectComponents(_rm,cd,dm,lp,addPartialEdge,addAllEdges);
#ifndef QUIET
        clock.StopClock();
    cout << clock.GetClock_SEC() << " sec, "
	 << GetCCcount(*(_rm->m_pRoadmap)) 
	 << " connected components\n"<< flush;
#endif
  }
}

#endif /*_ConnectMap_h_*/
