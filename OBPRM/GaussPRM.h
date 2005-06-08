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
  ///Destructor.
  virtual ~GaussPRM();

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
char*
GaussPRM<CFG>::
GetName() {
  return "GaussPRM";
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
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric *,
	      vector<CFG>& nodes) {
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

  if (gauss_d.GetValue() == 0) {  //if no Gauss_d value given, calculate from env
    gauss_d.PutValue(_env->Getminmax_BodyAxisRange());
  }
  
  
  // generate in bounding box
  for (int i=0,newNodes=0; i < numNodes.GetValue() || newNodes<1 ; i++) {
    
    // cfg1 & cfg2 are generated to be inside bbox
    CFG cfg1, cfg2;
    cfg1.GetRandomCfg(_env);
    cfg2.GetRandomCfg(_env);
    cfg2.c1_towards_c2(cfg1,cfg2,gauss_d.GetValue());
    
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
	
      } else if (!cfg1_free && cfg2_free) {
	nodes.push_back(CFG(cfg2));   
	newNodes++;
#if INTERMEDIATE_FILES
	path.push_back(cfg2);
#endif
	
      } else if (bExact){
	i--; // in this case, keep trying to generate the ith node;
	continue;
      }// endif push nodes
      
    } else if (bExact){ //outside of BBxz
	i--; // in this case, keep trying to generate the ith node;
    } // endif BB
    
  } // endfor
	
#if INTERMEDIATE_FILES
  WritePathConfigurations("GaussPRM.path", path, _env);
#endif

};

#endif
