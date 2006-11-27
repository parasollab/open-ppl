#ifndef BridgeTestPRM_h
#define BridgeTestPRM_h

#include "NodeGeneratorMethod.h"

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


  //////////////////////////////////////////////////
  //Gaussian number generator
  //parameters: m = median given by user
  //and s = standar deviation set to 0.5 for testSerial env.

  double GaussianDistribution(double m, double s);


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
double
BridgeTestPRM<CFG>::
GaussianDistribution(double m, double s){
  // normal distribution with mean m and standard deviation s
  double x1, x2, w, r,r1;
  do {
    r1 =(double)rand();
    r = ((double)(r1 / (RAND_MAX+1)))*-1;
    //cout<<"1. RAND_MAX::"<<RAND_MAX<<";R1::"<<r1<<"; R::"<<r<<endl;
    x1 = 2. * r - 1.;
    r1 =(double)rand();
    r = ( (double)(r1 / (RAND_MAX+1)) )*-1;
    //cout<<"2. RAND_MAX::"<<RAND_MAX<<";R1::"<<r1<<"; R::"<<r<<endl;
    x2 = 2. * r - 1.;
    w = x1*x1 + x2*x2;
    //cout<<"W: "<<w<<endl;
}while (w >= 1. || w < 1E-30);
 
  w = sqrt((-2.*log(w))/w);
  x1 *= w;
  return x1 * s + m;
}


template <class CFG>
void
BridgeTestPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
        CollisionDetection* cd, DistanceMetric* dm,
        vector<CFG>& nodes) {

  //DEBUG DATA
  int fail1(0),fail2(0),fail3(0);
  

  //LOG_DEBUG_MSG("BridgeTestPRM::GenerateNodes()");

  if (bridge_d.GetValue() == 0) {  //if no bridge_d value given (standard deviation), calculate from robot
    bridge_d.PutValue((_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange());
  } 

#ifndef QUIET
  cout << "(numNodes=" << this->numNodes.GetValue() << ") ";
  cout << "(chunkSize=" << this->chunkSize.GetValue() << ") ";
  cout << "(exactNodes=" << this->exactNodes.GetValue() << ") ";
  cout << "(d=" << bridge_d.GetValue() << ") ";
#endif
  
#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(this->numNodes.GetValue());
#endif
  bool bExact = this->exactNodes.GetValue() == 1? true: false;

  std::string Callee(GetName()), CallCnt;
  {std::string Method("-BridgeTestPRM::GenerateNodes"); Callee = Callee+Method;}

  int debug_attempts;
  // generate in bounding box
  for (int attempts=0,newNodes=0,success_cntr=0;  success_cntr < this->numNodes.GetValue() ; attempts++) {
    // cfg1 & cfg2 are generated to be inside bbox
    CFG cfg1, cfg2, cfgP, incr;

    cfg1.GetRandomCfg(_env);

    CallCnt="1"; 
    std::string tmpStr = Callee+CallCnt;
    
    bool cfg1_free = !cfg1.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
    //bool cfg1_bbox = cfg1.InBoundingBox(_env);
//    cout << "Attempting to find 1st collision node" << endl;
    if (!cfg1_free){
//    cout << "Sucessfully found 1st collision node" << endl;
      double gauss_mean = fabs(bridge_d.GetValue());
      double gauss_std = sqrt(gauss_mean);
      double gauss_dist = fabs(GaussianDistribution( gauss_mean, gauss_mean));
 //     double gauss_dist = fabs(randgauss(0,2*gauss_mean,gauss_mean,gauss_mean));
      //cout << "gauss_dist = " << gauss_dist << endl;
      
//      cout << "Got gass dist"<< endl;
      //cfg2.c1_towards_c2(cfg1,cfg2,gauss_dist);
//      cout << "Calling GetRandomRay where d = " << gauss_dist << endl;
        incr.GetRandomRay(gauss_dist, _env, dm);
//      cout << "Got Random ray" << endl;
      cfg2.add(cfg1, incr);

      CallCnt="2";
      tmpStr = Callee+CallCnt; 

      bool cfg2_free = !cfg2.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
//      cout << "Cfg::isCollision" << endl;
      bool cfg2_bbox = cfg2.InBoundingBox(_env);
//      cout << "Cfg::InBBox" << endl;
//     cout << "Attempting to find 2st collision node" << endl;
      if (!cfg2_free && cfg2_bbox) {
//      cout << "Sucessfully found 2st collision node" << endl;
        cfgP.WeightedSum(cfg1,cfg2,0.5);
        CallCnt="3";
        tmpStr = Callee+CallCnt; 

        bool cfgP_free = !cfgP.isCollision(_env,Stats,cd,*this->cdInfo, true, &tmpStr);
        bool cfgP_bbox = cfgP.InBoundingBox(_env);
//        cout << "Attempting to find free midpoint" << endl;
        if(cfgP_free && cfgP_bbox) {
//        cout << "Sucessfully found free midpoint" << endl;
          nodes.push_back(CFG(cfgP)); 
          newNodes++;
#if INTERMEDIATE_FILES
          path.push_back(cfgP);
#endif
        } else {++fail3;/*cout << "Failed to find free midpoint" << endl;*/}
      } else {++fail2;/*cout << "Failed to find 2nd collision node ";if(!cfg2_bbox) {cout << "b/c out of bbox" << endl;} else {cout << "b/c not in coll"<<endl; } */}
    } else {++fail1;/*cout<<"Failed to find 1st collision node"<< endl;*/}
    if (bExact)
      success_cntr = newNodes;
    else
      success_cntr = attempts+1;
    debug_attempts = attempts;
  } // endfor

 // cout << "BRIDGE_TEST_FINISHED:: attempts = "<< debug_attempts << ", fail1 = " << fail1;
 // cout << ", fail2 = " << fail2 << ", fail3 = " << fail3 << endl;

#if INTERMEDIATE_FILES
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
