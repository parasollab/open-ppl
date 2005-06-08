#ifndef CSpaceMAPRM_h
#define CSpaceMAPRM_h

#include "NodeGenerationMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class CSpaceMAPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates nodes approximately on the medial axis of the free C-space.  It is 
 *applicable to all types of cfgs.  This class is derived off of NodeGenerationMethod.
 */
template <class CFG>
class CSpaceMAPRM : public NodeGenerationMethod<CFG> {
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
  CSpaceMAPRM();
  ///Destructor.
  virtual ~CSpaceMAPRM();

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

  /*Generates random configurations and pushes them to the medial
   *axis of the C-Space. It considers both free nodes and nodes in
   *collision.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);

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
int CSpaceMAPRM<CFG>::nextNodeIndex = 0;


/////////////////////////////////////////////////////////////////////
//
//  definitions for class CSpaceMAPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
CSpaceMAPRM<CFG>::
CSpaceMAPRM() : NodeGenerationMethod<CFG>(),
  clearanceNum     ("clearance",         5,  1,   100),
  penetrationNum   ("penetration",       5,  1,   100) {
  clearanceNum.PutDesc("INT  ","(number of rays for approx clearance calulation, default 5)");
  penetrationNum.PutDesc("INT  ","(number of rays for approx penetration calculation, default 5)");
}


template <class CFG>
CSpaceMAPRM<CFG>::
~CSpaceMAPRM() {
}


template <class CFG>
char*
CSpaceMAPRM<CFG>::
GetName() {
  return "CSpaceMAPRM";
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  clearanceNum.PutValue(5);
  penetrationNum.PutValue(5);
}


template <class CFG>
int
CSpaceMAPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  for (int i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if (exactNodes.AckCmdLine(&i, argc, argv) ) {
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
CSpaceMAPRM<CFG>::
PrintUsage(ostream& _os) {
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; chunkSize.PrintUsage(_os);
  _os << "\n\t"; exactNodes.PrintUsage(_os);
  _os << "\n\t"; clearanceNum.PrintUsage(_os);
  _os << "\n\t"; penetrationNum.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << chunkSize.GetFlag() << " " << chunkSize.GetValue() << " ";
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue() << " ";
  _os << clearanceNum.GetFlag() << " " << clearanceNum.GetValue() << " ";
  _os << penetrationNum.GetFlag() << " " << penetrationNum.GetValue() << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
CSpaceMAPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new CSpaceMAPRM<CFG>(*this);
  return _copy;
}


template <class CFG>
void 
CSpaceMAPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats, CollisionDetection* cd, 
	      DistanceMetric *dm, vector<CFG>& nodes) {
#ifndef QUIET
  cout << "(numNodes="      << numNodes.GetValue()       << ", ";
  cout << "(chunkSize="      << chunkSize.GetValue()       << ", ";
  cout << "(exactNodes="      << exactNodes.GetValue()       << ", ";
  cout << "clearanceNum="   << clearanceNum.GetValue()   << ", ";
  cout << "penetrationNum=" << penetrationNum.GetValue() << ") ";
#endif
  
#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(numNodes.GetValue());
#endif
  bool bExact = exactNodes.GetValue() == 1? true: false;

  std::string Callee(GetName());
  {std::string Method("-cspaceMAPRM::GenerateNodes"); Callee = Callee+Method;}
  
  // MAPRM style node generation using clearances in the CSpace
  for (int i=0; i < numNodes.GetValue(); i++) {
    CFG cfg;
    cfg.GetMedialAxisCfg(_env,Stats,cd,*cdInfo,dm,
			 clearanceNum.GetValue(),penetrationNum.GetValue());
    //if in collision and asking for exactNum, keep generating
    
    if (! cfg.isCollision(_env,Stats,cd,*cdInfo) ){
      nodes.push_back(CFG(cfg));
#if INTERMEDIATE_FILES
      path.push_back(cfg);
#endif
    }else if (bExact){ // keep generating the ith node
      i --;
    }
  }
  
#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("csmaprm.path", path, _env);
#endif
}

#endif


