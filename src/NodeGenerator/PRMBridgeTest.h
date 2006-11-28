#ifndef BridgeTestPRM_h
#define BridgeTestPRM_h

#include "NodeGeneratorMethod.h"
#include "GaussianSamplers.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class BridgeTestPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes and filters them in a "gaussian" way.  This class is derived 
 *off of NodeGenerationMethod.
 */
template <class CFG>
class BridgeTestPRM: public NodeGenerationMethod<CFG> {
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
  BridgeTestPRM();
  BridgeTestPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ///Destructor.
  virtual ~BridgeTestPRM();

  //@}


  //////////////////////
  // Access
  virtual char* GetName();
  virtual void SetDefault();
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

  /**Generates a "bridge" (line segment) whose end points are cfgs in collision 
   *increases sampling density inside narrow passages
   *Brief Alg is:
   *   -# for i = 1 to n
   *       -# randomly generate cfg1
   *       -# if cfg1 is in collision
   *           -# pick a cfg2 near to cfg1 according to a gaussian  density fucntion
   *           -# if cfg2 is in collision
   *              -# set cgfP to be the midpoint of line segment cfg1&cgf2
   *              -# if cfgP is not in collision
   *                 -# add cfgP to the roadmap
   *              -# endif
   *           -# endif
   *       -# endif
   *   -# endfor
   *@param _env Used to get free Cfg.
   *@param cd Used to get free Cfg
   *@param nodes Used to store generated nodes.
   *
   *@see See Cfg::GetRandomCfg to know how to generate "one" random Cfg.
   *See CollisionDetection::isCollision to check collsion.
   *
   *@note If INTERMEDIATE_FILES is defined WritePathConfigurations will be
   *called.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class&,
           CollisionDetection* cd, 
           DistanceMetric *dm, vector<CFG>& nodes);

  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs);

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  /**Distance from surface
   */
  num_param<double> bridge_d;
  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;
};


template <class CFG>
int BridgeTestPRM<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class BridgeTestPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
BridgeTestPRM<CFG>::
BridgeTestPRM() : NodeGenerationMethod<CFG>(),
  bridge_d          ("d",                 0,  0,   5000000) {
  bridge_d.PutDesc("FLOAT  ","(distance, default based on environment)");
}


template <class CFG>
BridgeTestPRM<CFG>::
~BridgeTestPRM() {
}

template <class CFG>
BridgeTestPRM<CFG>::
BridgeTestPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_pNode, in_pProblem), 
  bridge_d          ("d",                 0,  0,   5000000) {
  LOG_DEBUG_MSG("BridgeTestPRM::BridgeTestPRM()");
  SetDefault();
  ParseXML(in_pNode);
  LOG_DEBUG_MSG("~BridgeTestPRM::BridgeTestPRM()");
}

template <class CFG>
char*
BridgeTestPRM<CFG>::
GetName() {
  return "BridgeTestPRM";
}

template <class CFG>
void
BridgeTestPRM<CFG>::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("BridgeTestPRM::ParseXML()");
//  SetDefault();
  if(!in_pNode) {
    LOG_ERROR_MSG("Error reading <shells> tag...."); exit(-1);
  }
  if(string(in_pNode->Value()) != "BridgeTestPRM") {
    LOG_ERROR_MSG("Error reading <BridgeTestPRM> tag...."); exit(-1);
  }
  double bridge;  
  in_pNode->ToElement()->QueryDoubleAttribute("bridge_d",&bridge);

  cout <<"BRIDGE DOULBE VALUE = " << bridge << endl;
  
  bridge_d.SetValue(bridge);
  
  
  PrintValues(cout);
  
  
  //PrintValues(cout);
  LOG_DEBUG_MSG("~BridgeTestPRM::ParseXML()");
}

template <class CFG>
void BridgeTestPRM<CFG>::
SetDefault() {
  //NodeGenerationMethod<CFG>::SetDefault();
  bridge_d.SetValue(0);
}

template <class CFG>
int
BridgeTestPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
BridgeTestPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
BridgeTestPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
BridgeTestPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  for (int i =1; i < argc; ++i) {
    if( this->numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if ( this->chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if ( this->exactNodes.AckCmdLine(&i, argc, argv) ) {
    }else if ( bridge_d.AckCmdLine(&i, argc, argv) ) {
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
BridgeTestPRM<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; this->numNodes.PrintUsage(_os);
  _os << "\n\t"; this->chunkSize.PrintUsage(_os);
  _os << "\n\t"; this->exactNodes.PrintUsage(_os);
  _os << "\n\t"; bridge_d.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
BridgeTestPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << this->numNodes.GetFlag() << " " << this->numNodes.GetValue() << " ";
  _os << this->chunkSize.GetFlag() << " " << this->chunkSize.GetValue() << " ";
  _os << this->exactNodes.GetFlag() << " " << this->exactNodes.GetValue() << " ";
  _os << bridge_d.GetFlag() << " " << bridge_d.GetValue() << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
BridgeTestPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new BridgeTestPRM<CFG>(*this);
  return _copy;
}

template <class CFG>
void
BridgeTestPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << this->numNodes.GetValue() << " ";
  out_os << " exact = " << this->exactNodes.GetValue() << " ";
  out_os << " chunk size = " << this->chunkSize.GetValue() << " ";
  out_os << " bridge d = " << bridge_d.GetValue() << " ";
  out_os << endl;
}


template <class CFG>
void
BridgeTestPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
        CollisionDetection* cd, DistanceMetric* dm,
        vector<CFG>& nodes) {
  LOG_DEBUG_MSG("BridgeTestPRM::GenerateNodes()");

  if (bridge_d.GetValue() == 0)   //if no bridge_d value given (standard deviation), calculate from robot
    bridge_d.PutValue((_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange());

#ifndef QUIET
  cout << "(numNodes=" << this->numNodes.GetValue() << ") ";
  cout << "(chunkSize=" << this->chunkSize.GetValue() << ") ";
  cout << "(exactNodes=" << this->exactNodes.GetValue() << ") ";
  cout << "(d=" << bridge_d.GetValue() << ") ";
#endif

  CDInfo cdInfo;
  BridgeTestRandomFreeSampler<CFG> bridge_sampler(_env, Stats, cd, cdInfo, dm, bridge_d.GetValue());
  int nodes_offset = nodes.size();

  for(int i=0; i<this->numNodes.GetValue(); ++i) {
    CFG tmp;
    tmp.GetRandomCfg(_env);
    if(this->exactNodes.GetValue() == 1)
      while(!bridge_sampler(tmp, nodes, 1)) {
        tmp.GetRandomCfg(_env);
      }
    else 
      bridge_sampler(tmp, nodes, 1);
  }

#if INTERMEDIATE_FILES
  vector<CFG> path; 
  copy(nodes.begin()+nodes_offset, nodes.end(),
       back_inserter<vector<CFG> >(path));
  WritePathConfigurations("BridgeTestPRM.path", path, _env);
#endif

  LOG_DEBUG_MSG("~BridgeTestPRM::GenerateNodes()"); 
}


template <class CFG>
void 
BridgeTestPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {
  LOG_DEBUG_MSG("BridgeTestPRM::GenerateNodes()"); 

  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass(); 
  CollisionDetection* pCd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric *dm = this->GetMPProblem()->GetDistanceMetric();
 
  GenerateNodes(pEnv,*pStatClass, pCd, dm, outCfgs);
}


#endif
