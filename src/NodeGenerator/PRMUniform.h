#ifndef BasicPRM_h
#define BasicPRM_h

#include "NodeGeneratorMethod.h"
#include "UniformSamplers.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class BasicPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes.  This class is derived off of NodeGenerationMethod.
 */
template <class CFG>
class BasicPRM: public NodeGenerationMethod<CFG> {
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
  BasicPRM();
  BasicPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.	
  virtual ~BasicPRM();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);
  virtual void ParseXML(XMLNodeReader& in_Node);

  //////////////////////
  // I/O methods
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os);
  virtual NodeGenerationMethod<CFG>* CreateCopy();

  /**Basic Randomized (probabilistic) Node Generation.
   *This method generates NodeGenerationMethod::numNodes collision-free Cfgs
   *and insert there Cfgs to nodes.
   *@param _env Used to get free Cfg.
   *@param cd Used to get free Cfg
   *@param nodes Used to store generated nodes.
   *@see See Cfg::GetFreeRandomCfg to know how to generate "one" free Cfg.
   *@note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
   *called.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     CollisionDetection* cd, 
			     DistanceMetric *dm, vector<CFG>& nodes);
  
  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs);

  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;
};


template <class CFG>
int BasicPRM<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class BasicPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
BasicPRM<CFG>::
BasicPRM() : NodeGenerationMethod<CFG>() {
}

template <class CFG>
BasicPRM<CFG>::
~BasicPRM() {
}


template <class CFG>
BasicPRM<CFG>::
BasicPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_Node, in_pProblem) {
  LOG_DEBUG_MSG("BasicPRM::BasicPRM()");
  ParseXML(in_Node);
  LOG_DEBUG_MSG("~BasicPRM::BasicPRM()");
}





template <class CFG>
char*
BasicPRM<CFG>::
GetName() {
  return "BasicPRM";
}

template <class CFG>
void
BasicPRM<CFG>::
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("BasicPRM::ParseXML()");
  PrintOptions(cout);
  LOG_DEBUG_MSG("~BasicPRM::ParseXML()");
}

template <class CFG>
int
BasicPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
BasicPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
BasicPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}



template <class CFG>
NodeGenerationMethod<CFG>* 
BasicPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BasicPRM<CFG>(*this);
  return _copy;
}

template <class CFG>
void
BasicPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << this->numNodes << " ";
  out_os << " exact = " << this->exactNodes << " ";
  out_os << " chunk size = " << this->chunkSize << " ";
  out_os << " MaxCDCalls = " << this->m_nMaxCdCalls << " ";
  out_os << endl;
}










template <class CFG>
void
BasicPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric *,
	      vector<CFG>& nodes) {
  LOG_DEBUG_MSG("BasicPRM::GenerateNodes()");	

#ifndef QUIET
  if (this->exactNodes==1)
     cout << "(numNodes=" << this->numNodes << ") ";
  else
    cout << "(exactNodes=" << this->exactNodes << ") ";
#endif
  
  CDInfo cdInfo;
  UniformRandomFreeSampler<CFG> uniform_sampler(_env, Stats, cd, cdInfo);
  int nodes_offset = nodes.size();

  if (this->exactNodes == 1) { // we want to obtain numNodes free nodes 
    for(int i=0; i<this->numNodes; ++i) {
      CFG tmp;
      if(!uniform_sampler(tmp, nodes, 100)) //default_maxTries = 100
        cerr << "Can't generate enough nodes!\n";
    }

  } else { //we want to try numNodess attempts (either free or in collision)
    for(int i=0; i<this->numNodes; ++i) {
      CFG tmp;
      uniform_sampler(tmp, nodes, 1); //attempts only
    }
  }

#if INTERMEDIATE_FILES
  vector<CFG> path;
  copy(nodes.begin()+nodes_offset, nodes.end(),
       back_inserter<vector<CFG> >(path));
  WritePathConfigurations("prm.path", path, _env);
#endif	
  
  LOG_DEBUG_MSG("~BasicPRM::GenerateNodes()"); 
}



template <class CFG>
void 
BasicPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {

/*** note, duplicate code here, should call UniformRandomFreeSampler instead ***/

  LOG_DEBUG_MSG("BasicPRM::GenerateNodes()"); 
  string callee("BasicPRM::GenerateNodes");
  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass();
  CollisionDetection* pCd = this->GetMPProblem()->GetCollisionDetection();
  
  if (this->exactNodes==1)
     cout << "(numNodes=" << this->numNodes << ") ";
  else
    cout << "(exactNodes=" << this->exactNodes << ") ";
  
  
  
  int nNumCdCalls = 0;
  if(this->m_nExactNodes == 1) {  //Generate exactly num nodes
    int nFree = 0;
    nNumCdCalls = 0;
    while(nFree < this->m_nNumNodes && nNumCdCalls < this->m_nMaxCdCalls) {
      CFG sample;
      sample.SetLabel("BasicPRM",true);
      sample.GetRandomCfg(pEnv);

      bool bCd = !sample.InBoundingBox(pEnv) || sample.isCollision(pEnv, *pStatClass, pCd, *this->cdInfo,true, &callee);
      ++nNumCdCalls;
      if (!bCd) {
        ++nFree;
        pStatClass->IncNodes_Generated();
        //tmp_pair.second = FREE;
      } else{
        ///\todo add a stats.IncColl call here later!
        //tmp_pair.second = COLL;
      }
      outCfgs.push_back(sample);
    }    
  } else {  //Attempt num nodes
    int nAttempts = 0;
    nNumCdCalls = 0;
    while(nAttempts < this->m_nNumNodes && nNumCdCalls < this->m_nMaxCdCalls) {
      CFG sample;
      sample.SetLabel("BasicPRM",true);
      sample.GetRandomCfg(pEnv);

      bool bCd = !sample.InBoundingBox(pEnv) || sample.isCollision(pEnv, *pStatClass, pCd, *this->cdInfo, true, &callee);
      ++nNumCdCalls;
      ++nAttempts;
      if (!bCd) {
        pStatClass->IncNodes_Generated();
        //tmp_pair.second = FREE;
      } else{
        ///\todo add a stats.IncColl call here later!
        //tmp_pair.second = COLL;
      }
      outCfgs.push_back(sample);
    }
  }
  
  LOG_DEBUG_MSG("~BasicPRM::GenerateNodes()"); 
  
  
};





#endif


