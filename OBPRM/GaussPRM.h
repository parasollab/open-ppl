#ifndef GaussPRM_h
#define GaussPRM_h

#include "NodeGenerationMethod.h"

//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class GaussPRM
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This generates PRM nodes and filters them in a "gaussian" way.  This class is derived 
 *off of NodeGenerationMethod.
 */
template <class CFG>
class GaussPRM: public NodeGenerationMethod<CFG> {
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
  GaussPRM();
  GaussPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem);
  ///Destructor.
  virtual ~GaussPRM();

  //@}


  //////////////////////////////////////////////////
  //Gaussian number generator
  //parameters: m = median given by user
  //and s = standar deviation set to 0.5 for testSerial env.

  double Gaussian(double m, double s);


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

  /**Filters randomly generated nodes in such a way that a "Gaussian" 
   *distribution on obstacle surfaces is retained.
   *Brief Alg is:
   *   -# for i = 1 to n
   *       -# randomly generate cfg1
   *       -# randomly generate cfg2 distance of "d" away from cfg1
   *       -# if one of (cfg1,cfg2) is in collision and the other is not
   *           -# add the free one to the roadmap
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
  /**Distance from surface to retain Gausian
   */
  num_param<double> gauss_d;
  //Index for next node 
  //used in incremental map generation
  static int nextNodeIndex;

};


template <class CFG>
int GaussPRM<CFG>::nextNodeIndex = 0;

/////////////////////////////////////////////////////////////////////
//
//  definitions for class GaussPRM declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
GaussPRM<CFG>::
GaussPRM() : NodeGenerationMethod<CFG>(),
  gauss_d          ("d",                 0,  0,   5000000) {
  gauss_d.PutDesc("FLOAT  ","(distance, default based on environment)");
}


template <class CFG>
GaussPRM<CFG>::
~GaussPRM() {
}

template <class CFG>
GaussPRM<CFG>::
GaussPRM(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_pNode, in_pProblem) {
  LOG_DEBUG_MSG("GaussPRM::GaussPRM()");
  ParseXML(in_pNode);
  LOG_DEBUG_MSG("~GaussPRM::GaussPRM()");
}


template <class CFG>
char*
GaussPRM<CFG>::
GetName() {
  return "GaussPRM";
}

template <class CFG>
void
GaussPRM<CFG>::
ParseXML(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("GaussPRM::ParseXML()");
  SetDefault();
  if(!in_pNode) {
    LOG_ERROR_MSG("Error reading <shells> tag...."); exit(-1);
  }
  if(string(in_pNode->Value()) != "GaussPRM") {
    LOG_ERROR_MSG("Error reading <GaussPRM> tag...."); exit(-1);
  }
  double gauss;  
  in_pNode->ToElement()->QueryDoubleAttribute("gauss_d",&gauss);
  
  gauss_d.SetValue(gauss);
  
  
  PrintValues(cout);
  LOG_DEBUG_MSG("~GaussPRM::ParseXML()");
}


template <class CFG>
void GaussPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  gauss_d.PutValue(0);
}


template <class CFG>
int
GaussPRM<CFG>::
GetNextNodeIndex() {
  return nextNodeIndex;
}

template <class CFG>
void
GaussPRM<CFG>::
SetNextNodeIndex(int index) {
  nextNodeIndex = index;
}

template <class CFG>
void
GaussPRM<CFG>::
IncreaseNextNodeIndex(int numIncrease) {
  nextNodeIndex += numIncrease;
}


template <class CFG>
void
GaussPRM<CFG>::
ParseCommandLine(int argc, char **argv) {
  for (int i =1; i < argc; ++i) {
    if( numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if ( chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if ( exactNodes.AckCmdLine(&i, argc, argv) ) {
    } else if ( gauss_d.AckCmdLine(&i, argc, argv) ) {
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
GaussPRM<CFG>::
PrintUsage(ostream& _os){
  _os.setf(ios::left,ios::adjustfield);
  
  _os << "\n" << GetName() << " ";
  _os << "\n\t"; numNodes.PrintUsage(_os);
  _os << "\n\t"; chunkSize.PrintUsage(_os);
  _os << "\n\t"; exactNodes.PrintUsage(_os);
  _os << "\n\t"; gauss_d.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
GaussPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << numNodes.GetFlag() << " " << numNodes.GetValue() << " ";
  _os << chunkSize.GetFlag() << " " << chunkSize.GetValue() << " ";
  _os << exactNodes.GetFlag() << " " << exactNodes.GetValue() << " ";
  _os << gauss_d.GetFlag() << " " << gauss_d.GetValue() << " ";
  _os << endl;
}


template <class CFG>
NodeGenerationMethod<CFG>* 
GaussPRM<CFG>::
CreateCopy() {
  NodeGenerationMethod<CFG> * _copy = new GaussPRM<CFG>(*this);
  return _copy;
}

template <class CFG>
void
GaussPRM<CFG>::
PrintOptions(ostream& out_os){
  out_os << "    " << GetName() << ":: ";
  out_os << " num nodes = " << numNodes.GetValue() << " ";
  out_os << " exact = " << exactNodes.GetValue() << " ";
  out_os << " chunk size = " << chunkSize.GetValue() << " ";
  out_os << endl;
}

template <class CFG>
double
GaussPRM<CFG>::
Gaussian(double m, double s){
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
GaussPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric *dm,
	      vector<CFG>& nodes) {

  LOG_DEBUG_MSG("GaussPRM::GenerateNodes()");	

#ifndef QUIET
  cout << "(numNodes=" << numNodes.GetValue() << ") ";
  cout << "(chunkSize=" << chunkSize.GetValue() << ") ";
  cout << "(exactNodes=" << exactNodes.GetValue() << ") ";
#endif

#if INTERMEDIATE_FILES
  vector<CFG> path; 
  path.reserve(numNodes.GetValue());
#endif
  bool bExact = exactNodes.GetValue() == 1? true: false;

  
  std::string Callee(GetName()), CallCnt;
  {std::string Method("-GaussPRM::GenerateNodes"); Callee = Callee+Method;}

  if (gauss_d.GetValue() == 0) {  //if no Gauss_d value given, calculate from robot
    gauss_d.PutValue((_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange());
  }
  
  //generate random number with normal distribution
  //this is the distance it will use to compute Cfg2

  double gauss_dist = Gaussian(gauss_d.GetValue(), 0.5); 
  
  // generate in bounding box
  //for (int i=0,newNodes=0; i < numNodes.GetValue() || newNodes<1 ; i++) {
  for (int attempts=0,newNodes=0,success_cntr=0;  success_cntr < numNodes.GetValue() ; attempts++) { 
    // cfg1 & cfg2 are generated to be inside bbox
    CFG cfg1, cfg2;
    cfg1.GetRandomCfg(_env);
    cfg2.GetRandomCfg(_env);
    //cfg2.c1_towards_c2(cfg1,cfg2,gauss_d.GetValue());
    cfg2.c1_towards_c2(cfg1,cfg2,gauss_dist);
    
    // because cfg2 is modified it must be checked again
    if (cfg2.InBoundingBox(_env)) {    
      CallCnt="1"; 
      std::string tmpStr = Callee+CallCnt;
      bool cfg1_free = !cfg1.isCollision(_env,Stats,cd,*cdInfo,true, &tmpStr);
      cfg1.obst = cdInfo->colliding_obst_index;
      
      CallCnt="2";
      tmpStr = Callee+CallCnt; 
      bool cfg2_free = !cfg2.isCollision(_env,Stats,cd,*cdInfo,true, &tmpStr);
      cfg2.obst = cdInfo->colliding_obst_index;
      
      if (cfg1_free && !cfg2_free) {
	nodes.push_back(CFG(cfg1));   
	newNodes++;
#if INTERMEDIATE_FILES
	path.push_back(cfg1);
#endif
	
      } 
      else if (!cfg1_free && cfg2_free) {
	nodes.push_back(CFG(cfg2));   
	newNodes++;
#if INTERMEDIATE_FILES
	path.push_back(cfg2);
#endif
      } 

/*       else if (bExact){ */
/* 	i--; // in this case, keep trying to generate the ith node; */
/* 	continue; */
/*       }// endif push nodes */
      
/*     }  */
/*     else if (bExact){ //outside of BBxz */
/*       i--; // in this case, keep trying to generate the ith node; */
/*     } // endif BB */
    }
    if (bExact)
      success_cntr = newNodes;
    else
      success_cntr = attempts+1;
    
  } // endfor
  
#if INTERMEDIATE_FILES
  WritePathConfigurations("GaussPRM.path", path, _env);
#endif

  LOG_DEBUG_MSG("~GaussPRM::GenerateNodes()"); 

}


template <class CFG>
void 
GaussPRM<CFG>::
GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG >  &outCfgs) {

  LOG_DEBUG_MSG("GaussPRM::GenerateNodes()"); 
 
  Environment* pEnv = in_pRegion;
  Stat_Class* pStatClass = in_pRegion->GetStatClass();
  CollisionDetection* pCd = GetMPProblem()->GetCollisionDetection();
  DistanceMetric *dm = GetMPProblem()->GetDistanceMetric();
 
  GenerateNodes(pEnv,*pStatClass, pCd, dm, outCfgs);

}


#endif
