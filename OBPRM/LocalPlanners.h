// $Id$
/////////////////////////////////////////////////////////////////////
/**@file  LocalPlanners.h
  *
  *    This set of classes supports a "Local Planning Algobase".
  *
  */

#ifndef LocalPlanners_h
#define LocalPlanners_h

//////////////////////////////////////////////////////////////////////////////////////////
#include "OBPRMDef.h"
#include "CollisionDetection.h" //for CDINFO instance, so we can not use forward declaration.
#include "Cfg.h"		//for vector<Cfg>, so we can not use forward declaration.
#include "Stat_Class.h"
#include "Weight.h"

extern Stat_Class Stats;

// Include LocalPlanners
#include "StraightLine.h"
#include "RotateAtS.h"
#include "ApproxSpheres.h"
#include "AStar.h" /*AStarDistance and AStarClearance defined here*/

class DistanceMetric;


template <class CFG, class WEIGHT>
struct LPOutput {
  vector<CFG> path;          ///< Path found by local planner.
  pair<WEIGHT, WEIGHT> edge; ///< Contains weights of edges defined in #path.
  vector< pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > > savedEdge;  ///< Failed Edge. savedEdge.second is the position that local planner failed.
};

   
template <class CFG, class WEIGHT>
class LocalPlanners {
 public:
  ///Default Constructor.
  LocalPlanners();
  ///Destructor.	
  ~LocalPlanners();

  //////////////////////
  // Access methods
  static vector<LocalPlannerMethod<CFG, WEIGHT> *> GetDefault();

  //////////////////////
  // I/O methods
  int ReadCommandLine(n_str_param* LPstrings[MAX_GN], int numLPs, cd_predefined _cdtype, SID _cdsetid, bool append_selected = false);
  bool ParseCommandLine(int argc, char **argv);
  bool InSelected(const LocalPlannerMethod<CFG,WEIGHT> *test_lp);
  void PrintUsage(ostream& _os);
  void PrintValues(ostream& _os);
  void WriteLPs(ostream& _os);
  void ReadLPs(istream& _is);

  void PrintDefaults(ostream& _os);

  unsigned int GetCounter();

  bool IsConnected(Environment *env, CollisionDetection *, DistanceMetric *,
		   CFG _c1, CFG _c2, LPOutput<CFG,WEIGHT>* lpOutput, 
		   double positionRes, double orientationRes, 
		   bool checkCollision=true, 
		   bool savePath=false, bool saveFailedPath=false);

  bool IsConnected(Roadmap<CFG, WEIGHT> *rm, 
		   CollisionDetection *cd, DistanceMetric *dm,
		   CFG _c1, CFG _c2, LPOutput<CFG, WEIGHT> *lpOutput,
		   double positionRes, double orientationRes, 
		   bool checkCollision=true, 
		   bool savePath=false, bool saveFailedPath=false);

  bool IsConnected(unsigned int lpid, Environment *_env, 
		   CollisionDetection *cd, 
		   DistanceMetric *dm,
		   CFG _c1, CFG _c2, LPOutput<CFG,WEIGHT>* lpOutput,
		   double positionRes, double orientationRes, 
		   bool checkCollision=true, 
		   bool savePath=false, bool saveFailedPath=false);

  bool UsesPlannerOtherThan(char plannerName[]);

  bool GetPathSegment(Environment *_env, CollisionDetection *cd, 
		      DistanceMetric *dm, CFG _c1, CFG _c2, WEIGHT _weight, 
		      LPOutput<CFG,WEIGHT>* _ci,	    
		      double positionRes, double orientationRes, 
		      bool checkCollision=true, 
		      bool savePath=false, bool saveFailedPath=false);

 protected:
  LocalPlannerMethod<CFG, WEIGHT> * GetLocalPlanner(unsigned int lpid); 
  unsigned int GetNewID();
  void ResetSelected();

  //////////////////////
  // Data
  vector<LocalPlannerMethod<CFG, WEIGHT>*> all;
  vector<LocalPlannerMethod<CFG, WEIGHT>*> selected;

  static int lp_counter;
  int saved_sl_id;
 public:
  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////


  /**@name */
  //@{
  /**Indicates what collision detector will be used.
   *@see CDSets for more information.
   */
  SID cdsetid;
  CDInfo cdInfo;
  //@}

  //Check the following variable against cdsetid
  static cd_predefined cdtype; ///< Used for building line segment. (lineSegmentInCollision)
  
};


template <class CFG, class WEIGHT>
int LocalPlanners<CFG,WEIGHT>::lp_counter = -1;

#if defined USE_CSTK
  template <class CFG, class WEIGHT> 
  cd_predefined LocalPlanners<CFG,WEIGHT>::cdtype = CSTK;
#elif defined USE_RAPID
  template <class CFG, class WEIGHT> 
  cd_predefined LocalPlanners<CFG,WEIGHT>::cdtype = RAPID;
#elif defined USE_PQP
  template <class CFG, class WEIGHT> 
  cd_predefined LocalPlanners<CFG,WEIGHT>::cdtype = PQP;
#elif defined USE_VCLIP
  template <class CFG, class WEIGHT> 
  cd_predefined LocalPlanners<CFG,WEIGHT>::cdtype = VCLIP;
#else
  #ifdef NO_CD_USE
    template <class CFG, class WEIGHT> 
    cd_predefined LocalPlanners<CFG,WEIGHT>::cdtype = CD_USER1;
  #else
    #error You have to specify at least one collision detection library.
  #endif
#endif

template <class CFG, class WEIGHT>
LocalPlanners<CFG,WEIGHT>::
LocalPlanners() {


#if defined USE_CSTK
  cdsetid = CSTK;
#elif defined USE_RAPID
  cdsetid = RAPID;
#elif defined USE_PQP
  cdsetid = PQP;
#elif defined USE_VCLIP
  cdsetid = VCLIP;
#else
#ifdef NO_CD_USE
  cdsetid = -1;
#else
#error You have to specify at least one collision detection library.
#endif
#endif

  StraightLine<CFG, WEIGHT>* straight_line = new StraightLine<CFG, WEIGHT>(cdtype);
  all.push_back(straight_line);

  RotateAtS<CFG, WEIGHT>* rotate_at_s = new RotateAtS<CFG,WEIGHT>(cdtype);
  all.push_back(rotate_at_s);
 
  AStarDistance<CFG, WEIGHT>* a_star_distance = new AStarDistance<CFG,WEIGHT>();
  all.push_back(a_star_distance);
  
  AStarClearance<CFG, WEIGHT>* a_star_clearance = new AStarClearance<CFG, WEIGHT>();
  all.push_back(a_star_clearance);
  
  ApproxSpheres<CFG, WEIGHT>* approx_spheres = new ApproxSpheres<CFG,WEIGHT>();
  all.push_back(approx_spheres);
  
  ResetSelected();
}

template <class CFG, class WEIGHT>
LocalPlanners<CFG,WEIGHT>::
~LocalPlanners() {
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    delete *I;
  
  for(I=all.begin(); I!=all.end(); I++)
    delete *I;
}

template <class CFG, class WEIGHT>
vector<LocalPlannerMethod<CFG, WEIGHT>*>
LocalPlanners<CFG,WEIGHT>::
GetDefault() {
  vector<LocalPlannerMethod<CFG, WEIGHT>*> Default;

  // in the command line says: default straightline rotate_at_s 0.5
  // in connectmapnodes it says: SL_R5
  // in the query it is: SL_R5_AD69, here I put SL_R5 by default.
  StraightLine<CFG, WEIGHT>* straight_line = new StraightLine<CFG, WEIGHT>(cdtype);
  Default.push_back(straight_line);
  
  RotateAtS<CFG, WEIGHT>* rotate_at_s = new RotateAtS<CFG, WEIGHT>(cdtype);
  Default.push_back(rotate_at_s);

/*    AStarDistance<CFG, WEIGHT>* a_star_distance = new AStarDistance<CFG,WEIGHT>();  */
/*    Default.push_back(a_star_distance); */

  return Default;
}

template <class CFG, class WEIGHT>
int
LocalPlanners<CFG,WEIGHT>::
ReadCommandLine(n_str_param* LPstrings[MAX_GN], int numLPs, cd_predefined _cdtype, SID _cdsetid, bool append_selected) {
  cdtype = _cdtype;
  cdsetid = _cdsetid;

  if (!append_selected)
    ResetSelected(); // reset lp_counter and clears selected vector

  for(int i=0; i<numLPs; i++) {

    istrstream _myistream(LPstrings[i]->GetValue());

    int argc = 0;
    char* argv[50];
    char cmdFields[50][100]; 
    while ( _myistream >> cmdFields[argc] ) {
      argv[argc] = (char*)(&cmdFields[argc]); 
      ++argc;
    }

    bool found = FALSE;
    try {
      found = ParseCommandLine(argc, argv);
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
    vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator itr;
    int lpid;
    selected = LocalPlanners<CFG,WEIGHT>::GetDefault();
    for (itr = selected.begin(), lpid = 1; itr != selected.end(); itr++, lpid++) {
      (*itr)->cdsetid = &cdsetid;
      (*itr)->cdInfo = &cdInfo;
      (*itr)->SetID(GetNewID());
    }
  }
  
}

template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
ParseCommandLine(int argc, char **argv) {
  bool found = FALSE;
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator itr;

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

	  vector<LocalPlannerMethod<CFG,WEIGHT>*>::iterator itr_names;
	  is_method_name = false;
	  for (itr_names = all.begin(); itr_names != all.end() &&cmd_begin+cmd_argc < argc; itr_names++)
	    if (!strcmp(argv[cmd_begin+cmd_argc],(*itr_names)->GetName())) {
	      is_method_name = true;
	      break;
	    }
	} while (! is_method_name && cmd_begin+cmd_argc < argc);

	// .. use the parser of the matching method
	(*itr)->ParseCommandLine(cmd_argc, cmd_argv);
	if (!InSelected(*itr)) {
	  // .., set their parameters
	  (*itr)->cdsetid = &cdsetid;
	  (*itr)->cdInfo = &cdInfo;
	  (*itr)->SetID(GetNewID());
	  //check if lp is straightline to set saved_sl_id for add partial edge
	  if ( !strcmp( "straightline", (*itr)->GetName()) )
	    saved_sl_id = (*itr)->GetID();
	  //  and push it back into the list of selected methods.
	  selected.push_back((*itr)->CreateCopy());
	}
      
	(*itr)->SetDefault(); //reset in all, not in selected
	found = TRUE;
	break;
      } 
    }
    cmd_begin = cmd_begin + cmd_argc;
  } while (cmd_begin < argc);
  return found;
}

template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
InSelected(const LocalPlannerMethod<CFG,WEIGHT> *test_lp) {
  bool result = false;
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for (I = selected.begin(); I != selected.end(); I++)
    if ( (*test_lp) == (**I) ) {
      result = true;
      break;
    }
  return result;
}

template <class CFG, class WEIGHT>
void 
LocalPlanners<CFG,WEIGHT>::
PrintUsage(ostream& _os) {
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for(I=all.begin(); I!=all.end(); I++)
    (*I)->PrintUsage(_os);
}

template <class CFG, class WEIGHT>
void 
LocalPlanners<CFG,WEIGHT>::
PrintValues(ostream& _os) {
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for(I=selected.begin(); I!=selected.end(); I++)
    (*I)->PrintValues(_os);
}

template <class CFG, class WEIGHT>
void 
LocalPlanners<CFG,WEIGHT>::
WriteLPs(ostream& _os) {
  _os << endl << "#####LPSTART#####";
  _os << endl << selected.size() << endl;  // number of lps
  PrintValues(_os);
  _os << "#####LPSTOP#####";
};

template <class CFG, class WEIGHT>
void 
LocalPlanners<CFG,WEIGHT>::
ReadLPs(istream& _is) {
	
  char tagstring[100];
  char lpdesc[100];
  int  numLPs;

  _is >> tagstring;
  if ( !strstr(tagstring,"LPSTART") ) {
    cout << endl << "In ReadLPs: didn't read LPSTART tag right";
    return;
  }
  
  _is >> numLPs;
  _is.getline(lpdesc,100,'\n');  // throw out rest of this line

  ResetSelected(); // reset lp_counter and clears selected vector

  for (int i = 0; i < numLPs; i++) {
    _is.getline(lpdesc,100,'\n');
    istrstream _lpstream(lpdesc);
    int argc = 0;
    char* argv[50];
    char cmdFields[50][100];
    while ( _lpstream >> cmdFields[argc] ) {
      argv[argc] = (char*)(&cmdFields[argc]);
      ++argc;
    }
    
    bool found = FALSE;
    try {
      found = ParseCommandLine(argc, argv);
      if (!found)
	throw BadUsage();
    } catch (BadUsage) {
      cerr << "Line error" << endl;
      exit(-1); 
    }
  }
  
  _is >> tagstring;
  if ( !strstr(tagstring,"LPSTOP") ) {
    cout << endl << "In ReadLPs: didn't read LPSTOP tag right";
    return;
  }
}


template <class CFG, class WEIGHT>
void
LocalPlanners<CFG,WEIGHT>::
PrintDefaults(ostream& _os) {
  vector<LocalPlannerMethod<CFG, WEIGHT>*> Default;
  Default = GetDefault();
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for(I=Default.begin(); I!=Default.end(); I++)
    (*I)->PrintValues(_os);
}


template <class CFG, class WEIGHT>
unsigned int 
LocalPlanners<CFG,WEIGHT>::
GetCounter() {
  return lp_counter;
}

//
// Find all LPs in the set that can make the connection
//
template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
IsConnected(Environment *_env, CollisionDetection *cd, DistanceMetric *dm,
	    CFG _c1, CFG _c2, LPOutput<CFG,WEIGHT>* lpOutput,
	    double positionRes, double orientationRes, 
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
  //clear lpOutput
  lpOutput->path.clear();      
  lpOutput->edge.first.SetWeight(0);
  lpOutput->edge.second.SetWeight(0);
  lpOutput->savedEdge.clear();

  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator itr;
  int connecting_lp = saved_sl_id;
  bool connected = false;

  if ( _c1.isWithinResolution(_c2, positionRes,orientationRes) ) { // *replace* with dm check
    //slt: the next two lines were only in IsConnected(Roadmap *,...) version
    if(!_c1.AlmostEqual(_c2)) {// avoid adding edge to itself.
      connected = true;
      connecting_lp = 1;
    }
  } else {
    for (itr = selected.begin(); !connected && itr != selected.end(); itr++) {
      lpOutput->path.erase(lpOutput->path.begin(),lpOutput->path.end());      
      lpOutput->edge.first.SetWeight(0);
      lpOutput->edge.second.SetWeight(0);
      
      connected = (*itr)->IsConnected(_env,cd,dm,_c1,_c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
      if (connected)
	connecting_lp = (*itr)->GetID();
    }
  }

  if ( !connected && !saveFailedPath ) {
    lpOutput->path.erase(lpOutput->path.begin(),lpOutput->path.end());
  }
  
  lpOutput->edge.first.SetLP(connecting_lp);
  lpOutput->edge.second.SetLP(connecting_lp);

  return connected;
};

// another overloaded implementation. this one check if the edge is already 
// in graph. So it will need a roadmap pointer instead of a pointer to env.
template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
IsConnected(Roadmap<CFG, WEIGHT> *rm, 
	    CollisionDetection *cd, DistanceMetric *dm,
	    CFG _c1, CFG _c2, LPOutput<CFG, WEIGHT> *lpOutput,
	    double positionRes, double orientationRes, 
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {

  bool connected;
  if( rm->m_pRoadmap->IsEdge(_c1, _c2) )  // check they are already connected.
    connected = true;
  else
    connected = IsConnected(rm->GetEnvironment(), cd, dm, _c1, _c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);

  return connected;
}


template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
IsConnected(unsigned int lpid, Environment *_env, CollisionDetection *cd, 
	    DistanceMetric *dm,
	    CFG _c1, CFG _c2, LPOutput<CFG,WEIGHT>* lpOutput,
	    double positionRes, double orientationRes, 
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
  return GetLocalPlanner(lpid)->IsConnected(_env,cd,dm,_c1,_c2, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
}


template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
UsesPlannerOtherThan(char plannerName[]){
  
  //Modified for VC
#if defined(_WIN32)
  using namespace std;
#endif
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator itr;
  for (itr = selected.begin(); itr != selected.end(); itr++) {
    cout << "\n\t UsesPlanner: " << (*itr)->GetName();
    if ( strcmp((*itr)->GetName(), plannerName) )
      return true;
  }
  return false;
}

template <class CFG, class WEIGHT>
bool
LocalPlanners<CFG,WEIGHT>::
GetPathSegment(Environment *_env, CollisionDetection *cd, DistanceMetric *dm, 
	       CFG _c1, CFG _c2, WEIGHT _weight, 
	       LPOutput<CFG,WEIGHT>* _ci,	    
	       double positionRes, double orientationRes, 
	       bool checkCollision, 
	       bool savePath, bool saveFailedPath) {
  bool connected = false;

  if (_weight.GetLP() >= 1 && _weight.GetLP() <= selected.size()) {         
    for (int lpid = _weight.GetLP(); !connected && lpid <= selected.size(); lpid++) {
      // clear possible old storage.	
      _ci->path.erase(_ci->path.begin(), _ci->path.end());
      //the local planner takes care of forward and backward connection
      if ( IsConnected(lpid, _env,cd,dm,_c1,_c2, _ci, positionRes, orientationRes, checkCollision, savePath, saveFailedPath) ) {
	connected = true;
      } else {                       // NEITHER!
	cout << "\n\n\t Planner: " << GetLocalPlanner(lpid)->GetName() << " FAILED!!! \n\n";
      }
    }
    
  } else { ///Local planner not found  
    cout << "\nERROR: _weight(" << _weight 
	 << ") is out of bounds (numLPs)" 
	 << "\n       where numLPs = " << selected.size()  << "\n";
  }
  return connected;
}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT> *
LocalPlanners<CFG,WEIGHT>::
GetLocalPlanner(unsigned int lpid) {
  LocalPlannerMethod<CFG, WEIGHT>* lp = NULL;
  if (lpid <= 0 || lpid > selected.size()) //out of bounds
    lp = NULL;
  else if (selected[lpid-1]->GetID() == lpid)
    lp = selected[lpid-1];
  else {   //Otherwise search the selected vector for the proper local planner
    vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator itr;
    for (itr = selected.begin(); itr != selected.end(); itr++) {
      if ( (*itr)->GetID() == lpid ) {
	lp = (*itr);
	break;
      }
    }
  }
  return lp;
}

template <class CFG, class WEIGHT>
unsigned int 
LocalPlanners<CFG,WEIGHT>::
GetNewID() {
  lp_counter++;
  return lp_counter;
}

template <class CFG, class WEIGHT>
void
LocalPlanners<CFG,WEIGHT>::
ResetSelected() {
  lp_counter = 0;
  saved_sl_id = -1;
  vector<LocalPlannerMethod<CFG, WEIGHT>*>::iterator I;
  for(I = selected.begin(); I != selected.end(); I++)
    delete *I;
  selected.clear();
}

#endif
