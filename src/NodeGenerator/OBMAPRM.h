#ifndef OBMAPRM_h
#define OBMAPRM_h

#include "OBPRM.h"
#include "ObstacleBasedSamplers.h"
#include "MedialAxisSamplers.h"

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
int OBMAPRM<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class OBMAPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
OBMAPRM<CFG>::
OBMAPRM() : OBPRM<CFG>()  {
  SetDefault();
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
  clearanceNum = 5;
  penetrationNum =5;
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
  cout << "(numNodes="          << this->numNodes          << ", ";
  cout << "chunkSize="          << this->chunkSize         << ", ";
  cout << "exactNodes="         << this->exactNodes         << ", ";
  cout << "clearanceNum="       << clearanceNum      << ", ";
  cout << "penetrationNum="     << penetrationNum    << ", ";
  cout << "\nproportionSurface="<< this->proportionSurface << ", ";
  cout << "\nnumShells="        << this->numShells         << ", ";
  cout << "collPair="           << this->collPair          << ", ";
  cout << "freePair="           << this->freePair          << ", ";
  cout << "clearanceFactor="    << this->clearanceFactor   << ") ";
#endif

  CDInfo cdInfo;
  ObstacleBasedSampler<CFG> ob_sampler(_env, Stats, cd, cdInfo, dm, this->numShells, 0);
  FreeMedialAxisSampler<CFG> ma_sampler(_env, Stats, cd, cdInfo, dm, clearanceNum, penetrationNum);
  int nodes_offset = nodes.size();

  int i=0; 
  while(i<this->numNodes) {
    CFG tmp;
    tmp.GetRandomCfg(_env);
    vector<CFG> in(1, tmp);
    vector<CFG> out1, out2;

    sample(ob_sampler, in.begin(), in.end(),
	   back_inserter<vector<CFG> >(out1), 1);
    sample(ma_sampler, out1.begin(), out1.end(),
           back_inserter<vector<CFG> >(out2), 1);

    copy(out2.begin(), out2.end(), back_inserter<vector<CFG> >(nodes));

    i += out2.size();
    //make sure advance if not exact
    if(out2.empty() && this->exactNodes == 0) 
      i++;
  }

  //correct any extras if exact true
  if((this->exactNodes == 1) && 
     ((nodes.size()-nodes_offset) > this->numNodes)) 
    nodes.erase(nodes.begin()+nodes_offset+this->numNodes,
	        nodes.end());

#ifdef INTERMEDIATE_FILES
  vector<CFG> path;
  copy(nodes.begin()+nodes_offset, nodes.end(),
       back_inserter<vector<CFG> >(path));
  WritePathConfigurations("obmaprm.path", path, _env);
#endif
}


#endif


