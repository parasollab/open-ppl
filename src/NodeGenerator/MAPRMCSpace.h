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
  CSpaceMAPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
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
  virtual void ParseXML(XMLNodeReader& in_Node) { };

  //////////////////////
  // I/O methods
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /*Generates random configurations and pushes them to the medial
   *axis of the C-Space. It considers both free nodes and nodes in
   *collision.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats, 
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
  int clearanceNum;
  /**Number of rays for approx penetration calculation
   *@see Cfg::ApproxCSpaceClearance
   */
  int penetrationNum;

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
CSpaceMAPRM() : NodeGenerationMethod<CFG>() {
  SetDefault();
}

template <class CFG>
CSpaceMAPRM<CFG>::
CSpaceMAPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) : NodeGenerationMethod<CFG>(in_Node,in_pProblem) {
  SetDefault();
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
  clearanceNum = 10; ///Modified for RoadmapComparision!!!! was 5.
  penetrationNum = 5;
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
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << "num nodes" << " " << this->numNodes << " ";
  out_os << "chunk size" << " " << this->chunkSize << " ";
  out_os << "exact" << " " << this->exactNodes << " ";
  out_os << "clearanceNum" << " " << clearanceNum << " ";
  out_os << "penetrationNum" << " " << penetrationNum << " ";
 // out_os << "validity method = " << this->vcMethod << " ";

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
GenerateNodes(Environment* _env, Stat_Class& Stats, 
	      DistanceMetric *dm, vector<CFG>& nodes) {
CollisionDetection* cd = this->GetMPProblem()->GetCollisionDetection();
#ifndef QUIET
  cout << "(numNodes="      << this->numNodes       << ", ";
  cout << "(chunkSize="      << this->chunkSize       << ", ";
  cout << "(exactNodes="      << this->exactNodes       << ", ";
  cout << "clearanceNum="   << clearanceNum   << ", ";
  cout << "penetrationNum=" << penetrationNum << ", ";
//  cout << "validity method=" << this->vcMethod << ") ";
#endif

  CDInfo cdInfo;
  FreeMedialAxisSampler<CFG> ma_sampler(_env, Stats, cd, cdInfo, dm, clearanceNum, penetrationNum);
  int nodes_offset = nodes.size();

  for(int i=0; i<this->numNodes; ++i) {
    CFG tmp;
    tmp.GetRandomCfg(_env);
    if(this->exactNodes == 1)
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
  DistanceMetric* dm =  this->GetMPProblem()->GetDistanceMetric();
  
  GenerateNodes(_env,  Stats, dm, nodes);
};


#endif


