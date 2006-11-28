#ifndef CSpaceMAPRM_h
#define CSpaceMAPRM_h

#include "NodeGeneratorMethod.h"
#include "MedialAxisSamplers.h"

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
  CSpaceMAPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem);
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
  virtual void ParseXML(TiXmlNode* in_pNode) { };

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /*Generates random configurations and pushes them to the medial
   *axis of the C-Space. It considers both free nodes and nodes in
   *collision.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);

  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG > &nodes);

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
CSpaceMAPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem) : NodeGenerationMethod<CFG>(in_pNode,in_pProblem),
  clearanceNum     ("clearance",         10,  1,   100),///Modified for RoadmapComparision!!!! was 5.
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

///\todo Fixback default
template <class CFG>
void
CSpaceMAPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  clearanceNum.PutValue(10); ///Modified for RoadmapComparision!!!! was 5.
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
    if( this->numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if (this->chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if (this->exactNodes.AckCmdLine(&i, argc, argv) ) {
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
  _os << "\n\t"; this->numNodes.PrintUsage(_os);
  _os << "\n\t"; this->chunkSize.PrintUsage(_os);
  _os << "\n\t"; this->exactNodes.PrintUsage(_os);
  _os << "\n\t"; clearanceNum.PrintUsage(_os);
  _os << "\n\t"; penetrationNum.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << this->numNodes.GetFlag() << " " << this->numNodes.GetValue() << " ";
  _os << this->chunkSize.GetFlag() << " " << this->chunkSize.GetValue() << " ";
  _os << this->exactNodes.GetFlag() << " " << this->exactNodes.GetValue() << " ";
  _os << clearanceNum.GetFlag() << " " << clearanceNum.GetValue() << " ";
  _os << penetrationNum.GetFlag() << " " << penetrationNum.GetValue() << " ";
  _os << endl;
}

template <class CFG>
void
CSpaceMAPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << this->numNodes.GetFlag() << " " << this->numNodes.GetValue() << " ";
  out_os << this->chunkSize.GetFlag() << " " << this->chunkSize.GetValue() << " ";
  out_os << this->exactNodes.GetFlag() << " " << this->exactNodes.GetValue() << " ";
  out_os << clearanceNum.GetFlag() << " " << clearanceNum.GetValue() << " ";
  out_os << penetrationNum.GetFlag() << " " << penetrationNum.GetValue() << " ";
  out_os << endl;
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
  cout << "(numNodes="      << this->numNodes.GetValue()       << ", ";
  cout << "(chunkSize="      << this->chunkSize.GetValue()       << ", ";
  cout << "(exactNodes="      << this->exactNodes.GetValue()       << ", ";
  cout << "clearanceNum="   << clearanceNum.GetValue()   << ", ";
  cout << "penetrationNum=" << penetrationNum.GetValue() << ") ";
#endif

  CDInfo cdInfo;
  FreeMedialAxisSampler<CFG> ma_sampler(_env, Stats, cd, cdInfo, dm, clearanceNum.GetValue(), penetrationNum.GetValue());
  int nodes_offset = nodes.size();

  for(int i=0; i<this->numNodes.GetValue(); ++i) {
    CFG tmp;
    tmp.GetRandomCfg(_env);
    if(this->exactNodes.GetValue() == 1)
      while(!ma_sampler(tmp, nodes, 1)) {
        tmp.GetRandomCfg(_env);
      }
    else
      ma_sampler(tmp, nodes, 1);
  }

#if INTERMEDIATE_FILES
  vector<CFG> path; 
  copy(nodes.begin()+nodes_offset, nodes.end(),
       back_inserter<vector<CFG> >(path));
  WritePathConfigurations("csmaprm.path", path, _env);
#endif  
}


template <class CFG>
void
CSpaceMAPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG > &nodes) {
  Environment* _env = in_pRegion;
  Stat_Class& Stats = *(in_pRegion->GetStatClass());
  CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric* dm =  this->GetMPProblem()->GetDistanceMetric();
  
  GenerateNodes(_env,  Stats,  cd,  dm, nodes);
};


#endif


