#ifndef OBMAPRM_h
#define OBMAPRM_h

#include "OBPRM.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class OBMAPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This is a hybrid between OBPRM and CSpaceMAPRM.  It first generates nodes with OBPRM,
 *and then it pushes to the medial axis of the free C-space.  It is 
 *applicable to all types of cfgs.  This class is derived off of OBPRM.
 */
template <class CFG>
class OBMAPRM : public OBPRM<CFG> {
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
  OBMAPRM();
  ///Destructor.
  virtual ~OBMAPRM();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();
  
  /*OBPRM-CSpaceMAPRM Hybrid.
   */
  virtual void GenerateNodes(Environment* _env, CollisionDetection* cd, 
			     DistanceMetric *, vector<CFG>& nodes);

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**Number of rays for approx clearance calculation
   *@see Cfg::ApproxCSpaceClearance
   */
  num_param<int> clearanceNum;
  /**Number of rays for approx penetration calculation
   *@see Cfg::ApproxCSpaceClearance
   */
  num_param<int> penetrationNum;
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class OBMAPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
OBMAPRM<CFG>::
OBMAPRM() : OBPRM<CFG>(),
  clearanceNum     ("clearance",         5,  1,   100),
  penetrationNum   ("penetration",       5,  1,   100)  {
  clearanceNum.PutDesc("INT  ","(number of rays for approx clearance calulation, default 5)");
  penetrationNum.PutDesc("INT  ","(number of rays for approx penetration calculation, default 5)");
}


template <class CFG>
OBMAPRM<CFG>::
~OBMAPRM() {
}


template <class CFG>
char*
OBMAPRM<CFG>::
GetName() {
  return "OBMAPRM";
}


template <class CFG>
void
OBMAPRM<CFG>::
SetDefault() {
  OBPRM<CFG>::SetDefault();
  clearanceNum.PutValue(5);
  penetrationNum.PutValue(5);
}


template <class CFG>
void
OBMAPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (numShells.AckCmdLine(&i, argc, argv) ) {
    } else if (proportionSurface.AckCmdLine(&i, argc, argv) ) {
    } else if (collPair.AckCmdLine(&i, argc, argv) ) {
      if (!ValidatePairs(NULL,collPair,NULL)) {
	cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	     << collPair.GetValue() <<"\"\n\n";
	PrintUsage(cerr);
	cerr << endl;
	exit (-1);
      }
    } else if (freePair.AckCmdLine(&i, argc, argv) ) {
      if (!ValidatePairs(NULL,freePair,NULL)) {
	cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	     << freePair.GetValue() <<"\"\n\n";
	PrintUsage(cerr);
	cerr << endl;
	exit (-1);
      }
    } else if (clearanceFactor.AckCmdLine(&i, argc, argv) ) {
    } else if (clearanceNum.AckCmdLine(&i, argc, argv) ) {
    } else if (penetrationNum.AckCmdLine(&i, argc, argv) ) {
    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	   << argv <<"\"\n\n";
      PrintUsage(cerr);
      cerr << endl;
      exit (-1);
    }
  }
}


template <class CFG>
void
OBMAPRM<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; clearanceNum.PrintUsage(_os);
  _os << "\n\t"; penetrationNum.PrintUsage(_os);
  _os << "\n\t"; numShells.PrintUsage(_os);
  _os << "\n\t"; proportionSurface.PrintUsage(_os);
  _os << "\n\t"; numShells.PrintUsage(_os);
  _os << "\n\t"; collPair.PrintUsage(_os);
  _os << "\n\t"; freePair.PrintUsage(_os);
  _os << "\n\t"; clearanceFactor.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
OBMAPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << clearanceNum.GetFlag() << " " << clearanceNum.GetValue() << " ";
  _os << penetrationNum.GetFlag() << " " << penetrationNum.GetValue() << " ";
  _os << numShells.GetFlag() << " " << numShells.GetValue() << " ";
  _os << proportionSurface.GetFlag() << " " << proportionSurface.GetValue() << " ";
  _os << collPair.GetFlag() << " " << collPair.GetValue() << " ";
  _os << freePair.GetFlag() << " " << freePair.GetValue() << " ";
  _os << clearanceFactor.GetFlag() << " " << clearanceFactor.GetValue() << " ";

 _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
OBMAPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new OBMAPRM<CFG>(*this);
  return _copy;
}


template <class CFG>
void
OBMAPRM<CFG>::
GenerateNodes(Environment* _env, CollisionDetection* cd, DistanceMetric* dm,
	      vector<CFG>& nodes) {
#ifndef QUIET
  cout << "(numNodes="          << numNodes.GetValue()          << ", ";
  cout << "clearanceNum="       << clearanceNum.GetValue()      << ", ";
  cout << "penetrationNum="     << penetrationNum.GetValue()    << ", ";
  cout << "\nproportionSurface="<< proportionSurface.GetValue() << ", ";
  cout << "\nnumShells="        << numShells.GetValue()         << ", ";
  cout << "collPair="           << collPair.GetValue()          << ", ";
  cout << "freePair="           << freePair.GetValue()          << ", ";
  cout << "clearanceFactor="    << clearanceFactor.GetValue()   << ") ";
#endif
  
#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(numNodes.GetValue());
#endif
  
  //generate obprm nodes   
  OBPRM<CFG>::GenerateNodes(_env,cd,dm,nodes);
  
  //copy nodes to obprmCfgs and erase nodes vector
  vector<CFG> obprmCfgs;
  int i;
  for (i=0; i < nodes.size(); i++) {
    obprmCfgs.push_back(nodes[i]);
  }
  nodes.clear();
  
  cout << "\nobprmnodes = " << obprmCfgs.size() << "\n";
  
  //push nodes to medial axis
  for (i=0; i < obprmCfgs.size(); i++) {
    CFG cfg = obprmCfgs[i];
    
    cfg.PushToMedialAxis(_env, cd, *cdInfo, 
			 dm, clearanceNum.GetValue(), 
			 penetrationNum.GetValue());
    
    if ( !cfg.isCollision(_env, cd, *cdInfo) ) {
      nodes.push_back(CFG(cfg));
#if INTERMEDIATE_FILES
      path.push_back(cfg);
#endif
    }
  }
  
#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("obmaprm.path", path, _env);
#endif
}

#endif


