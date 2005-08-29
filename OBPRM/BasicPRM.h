#ifndef BasicPRM_h
#define BasicPRM_h

#include "NodeGenerationMethod.h"

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
  BasicPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ///Destructor.	
  virtual ~BasicPRM();

  //@}

  //////////////////////
  // Access
  virtual char* GetName();
  virtual int GetNextNodeIndex();
  virtual void SetNextNodeIndex(int);
  virtual void IncreaseNextNodeIndex(int);
  virtual void ParseXML(TiXmlNode* in_pNode);

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv);
  virtual void PrintUsage(ostream& _os);
  virtual void PrintValues(ostream& _os);
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
BasicPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_pNode, in_pProblem) {
  LOG_DEBUG_MSG("BasicPRM::BasicPRM()");
  ParseXML(in_pNode);
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
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("BasicPRM::ParseXML()");
  PrintValues(cout);
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
void
BasicPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  int i;
  for (i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) || 
        chunkSize.AckCmdLine(&i, argc, argv) ||
	exactNodes.AckCmdLine(&i, argc, argv)) {
/*         numAttempts.AckCmdLine(&i, argc, argv) || */
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
BasicPRM<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os); _os << " ";
  _os << "\n\t"; chunkSize.PrintUsage(_os); _os << " ";
/*   _os << "\n\t"; numAttempts.PrintUsage(_os); */
  _os << "\n\t"; exactNodes.PrintUsage(_os);


  _os.setf(ios::right,ios::adjustfield);
}

template <class CFG>
void
BasicPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << chunkSize.GetFlag() << " " <<chunkSize.GetValue() << " ";

/*   _os << numAttempts.GetFlag() << " " << numAttempts.GetValue(); */
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue();
  _os << endl;
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
  out_os << " num nodes = " << numNodes.GetValue() << " ";
  out_os << " exact = " << exactNodes.GetValue() << " ";
  out_os << " chunk size = " << chunkSize.GetValue() << " ";
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
  if (exactNodes.GetValue()==1)
     cout << "(numNodes=" << numNodes.GetValue() << ") ";
  else
    cout << "(exactNodes=" << exactNodes.GetValue() << ") ";
#endif
  
  //PRM style node generation -- generate in expanded bounding box
  vector<Cfg*> path;
  
  if (exactNodes.GetValue() == 1) { // we want to obtain numNodes free nodes 
    CFG tmp;
    int default_maxTries = 100;
    for (int i=0; i < numNodes.GetValue(); ++i) {
       bool collision = true;
       int j=0;
       while (collision && (j<default_maxTries)) {
         j++;
         Stats.IncNodes_Attempted();
         tmp.GetRandomCfg(_env);
         if (!tmp.isCollision(_env, Stats, cd, *cdInfo)) {
	   nodes.push_back( CFG(tmp));
	   path.push_back ( (Cfg*)tmp.CreateNewCfg() );
	   Stats.IncNodes_Generated();
           collision = false;
         }
//#ifdef COLLISIONCFG
//	 else{
//	   m_vGeneratedCollisionConfiguration[cdInfo->colliding_obst_index].push_back(tmp);
//	 }
//#endif

       }
       if (j == default_maxTries)
	 cerr << "Can't generate enought nodes! " << endl;
    }
  } else { //we want to try numNodess attempts (either free or in collision)
    CFG tmp;
    for (int i=0; i < numNodes.GetValue(); ++i) {
      Stats.IncNodes_Attempted();
      tmp.GetRandomCfg(_env);
      if (!tmp.isCollision(_env, Stats, cd, *cdInfo)) {
	nodes.push_back( CFG(tmp));
	path.push_back ( (Cfg*)tmp.CreateNewCfg() );
	Stats.IncNodes_Generated();
      }
#ifdef COLLISIONCFG
      else{
	m_vGeneratedCollisionConfiguration[cdInfo->colliding_obst_index].push_back(tmp);
      }
#endif
    }
  }
  
#if INTERMEDIATE_FILES
  //in util.h
  WritePathConfigurations("prm.path", path, _env);
#endif	
  int i;
  for(i=0; i<path.size(); i++)
    if (path[i]!=NULL)
      delete path[i];
  
  
  LOG_DEBUG_MSG("~BasicPRM::GenerateNodes()"); 
}



template <class CFG>
void 
BasicPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {

  LOG_DEBUG_MSG("BasicPRM::GenerateNodes()"); 
  
  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass();
  CollisionDetection* pCd = GetMPProblem()->GetCollisionDetection();
  
  if (exactNodes.GetValue()==1)
     cout << "(numNodes=" << numNodes.GetValue() << ") ";
  else
    cout << "(exactNodes=" << exactNodes.GetValue() << ") ";
  
  
  
  int nNumCdCalls = 0;
  if(m_nExactNodes == 1) {  //Generate exactly num nodes
    int nFree = 0;
    nNumCdCalls = 0;
    while(nFree < m_nNumNodes && nNumCdCalls < m_nMaxCdCalls) {
      CFG sample;
      sample.SetLabel("BasicPRM",true);
      sample.GetRandomCfg(pEnv);
      bool bCd = sample.isCollision(pEnv, *pStatClass, pCd, *cdInfo);
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
    while(nAttempts < m_nNumNodes && nNumCdCalls < m_nMaxCdCalls) {
      CFG sample;
      sample.SetLabel("BasicPRM",true);
      sample.GetRandomCfg(pEnv);
      bool bCd = sample.isCollision(pEnv, *pStatClass, pCd, *cdInfo);
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


