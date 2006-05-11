#ifndef OBMAPRM_h
#define OBMAPRM_h

#include "OBPRM.h"

#define MAX_NUM_NODES_TRIES 100

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
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();
  
  /*OBPRM-CSpaceMAPRM Hybrid.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats, 
			     CollisionDetection* cd, 
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

  //Index for next node
  //used in incremental map generation
  static int nextNodeIndex;

};


template <class CFG>
int OBMAPRM<CFG>::nextNodeIndex = 0;

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
int
OBMAPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}


template <class CFG>
void
OBMAPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}


template <class CFG>
void
OBMAPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
OBMAPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( this->numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (this->chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if (this->exactNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (this->numShells.AckCmdLine(&i, argc, argv) ) {
    } else if (this->proportionSurface.AckCmdLine(&i, argc, argv) ) {
    } else if (this->collPair.AckCmdLine(&i, argc, argv) ) {
      if (!this->ValidatePairs(NULL,this->collPair,NULL)) {
	cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	     << this->collPair.GetValue() <<"\"\n\n";
	PrintUsage(cerr);
	cerr << endl;
	exit (-1);
      }
    } else if (this->freePair.AckCmdLine(&i, argc, argv) ) {
      if (!this->ValidatePairs(NULL,this->freePair,NULL)) {
	cerr << "\nERROR ParseCommandLine: Don\'t understand \""
	     << this->freePair.GetValue() <<"\"\n\n";
	PrintUsage(cerr);
	cerr << endl;
	exit (-1);
      }
    } else if (this->clearanceFactor.AckCmdLine(&i, argc, argv) ) {
    } else if (clearanceNum.AckCmdLine(&i, argc, argv) ) {
    } else if (penetrationNum.AckCmdLine(&i, argc, argv) ) {
    } else {
      cerr << "\nERROR ParseCommandLine: Don\'t understand \"";
      for(int j=0; j<argc; j++)
        cerr << argv[j] << " ";
      cerr << "\"\n\n";
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
  _os << "\n\t"; this->numNodes.PrintUsage(_os);
  _os << "\n\t"; this->chunkSize.PrintUsage(_os);
  _os << "\n\t"; this->exactNodes.PrintUsage(_os);
  _os << "\n\t"; clearanceNum.PrintUsage(_os);
  _os << "\n\t"; penetrationNum.PrintUsage(_os);
  _os << "\n\t"; this->proportionSurface.PrintUsage(_os);
  _os << "\n\t"; this->numShells.PrintUsage(_os);
  _os << "\n\t"; this->collPair.PrintUsage(_os);
  _os << "\n\t"; this->freePair.PrintUsage(_os);
  _os << "\n\t"; this->clearanceFactor.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
OBMAPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << this->numNodes.GetFlag() << " " << this->numNodes.GetValue() << " ";
  _os << this->chunkSize.GetFlag() << " " << this->chunkSize.GetValue() << " ";
  _os << this->exactNodes.GetFlag() << " " << this->exactNodes.GetValue() << " ";
  _os << clearanceNum.GetFlag() << " " << clearanceNum.GetValue() << " ";
  _os << penetrationNum.GetFlag() << " " << penetrationNum.GetValue() << " ";
  _os << this->numShells.GetFlag() << " " << this->numShells.GetValue() << " ";
  _os << this->proportionSurface.GetFlag() << " " << this->proportionSurface.GetValue() << " ";
  _os << this->collPair.GetFlag() << " " << this->collPair.GetValue() << " ";
  _os << this->freePair.GetFlag() << " " << this->freePair.GetValue() << " ";
  _os << this->clearanceFactor.GetFlag() << " " << this->clearanceFactor.GetValue() << " ";

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
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      CollisionDetection* cd, DistanceMetric* dm,
	      vector<CFG>& nodes) {
#ifndef QUIET
  cout << "(numNodes="          << this->numNodes.GetValue()          << ", ";
  cout << "chunkSize="          << this->chunkSize.GetValue()         << ", ";
  cout << "exactNodes="         << this->exactNodes.GetValue()         << ", ";
  cout << "clearanceNum="       << clearanceNum.GetValue()      << ", ";
  cout << "penetrationNum="     << penetrationNum.GetValue()    << ", ";
  cout << "\nproportionSurface="<< this->proportionSurface.GetValue() << ", ";
  cout << "\nnumShells="        << this->numShells.GetValue()         << ", ";
  cout << "collPair="           << this->collPair.GetValue()          << ", ";
  cout << "freePair="           << this->freePair.GetValue()          << ", ";
  cout << "clearanceFactor="    << this->clearanceFactor.GetValue()   << ") ";
#endif

  bool bExact = this->exactNodes.GetValue() == 1? true: false;
  
#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(this->numNodes.GetValue());
#endif

  std::string Callee(GetName());
  {std::string Method("OBMAPRM::GenerateNodes");Callee=Callee+Method;}
  
  int nNodesGoal = this->numNodes.GetValue();
  int nNodesGap = bExact ? nNodesGoal - nodes.size() : nNodesGoal;
  //generate obprm nodes   
  int nNumTries = 0;
  while (nNodesGap >0 && nNumTries < MAX_NUM_NODES_TRIES){
    vector<CFG> obprmCfgs;
    this->numNodes.PutValue(nNodesGap);
    OBPRM<CFG>::GenerateNodes(_env,Stats,cd,dm,obprmCfgs);
    
/*   //copy nodes to obprmCfgs and erase nodes vector */
/*   int i; */
/*   for (i=0; i < nodes.size(); i++) { */
/*     obprmCfgs.push_back(nodes[i]); */
/*   } */
/*   nodes.clear(); */
    cout << "\nobprmnodes = " << obprmCfgs.size() << "\n";
  
  //push nodes to medial axis
    for (int i=0; i < obprmCfgs.size(); i++) {
      CFG cfg = obprmCfgs[i];
    
      cfg.PushToMedialAxis(_env, Stats, cd, *this->cdInfo, 
			   dm, clearanceNum.GetValue(), 
			   penetrationNum.GetValue());
    
      if (cfg.InBoundingBox(_env) && !cfg.isCollision(_env, Stats, cd, *this->cdInfo,true, &Callee) ) {
	nodes.push_back(CFG(cfg));
#if INTERMEDIATE_FILES	
	path.push_back(cfg);
#endif	
      }
    }
  
    nNodesGap = bExact? nNodesGoal - nodes.size() : 0;
    nNumTries ++;
  } // while (nNodesGap >0)

  if (nNumTries >= MAX_NUM_NODES_TRIES)
    cerr << GetName() << ": Can\'t generate engough nodes! " << endl;

  this->numNodes.PutValue(nNodesGoal);//restore the original value - not sure if necessary.
#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("obmaprm.path", path, _env);
#endif
}

#endif


