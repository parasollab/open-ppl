#ifndef GaussPRM_h
#define GaussPRM_h

#include "NodeGeneratorMethod.h"
#include "GaussianSamplers.h"

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
NodeGenerationMethod<CFG>(in_pNode, in_pProblem),
gauss_d          ("d",                 0,  0,   5000000) {
  LOG_DEBUG_MSG("GaussPRM::GaussPRM()");
  gauss_d.PutDesc("FLOAT  ","(distance, default based on environment)");
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
 // SetDefault();
  gauss_d.PutValue(0);
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
    if( this->numNodes.AckCmdLine(&i, argc, argv) ) {
    } else if ( this->chunkSize.AckCmdLine(&i, argc, argv) ) {
    } else if ( this->exactNodes.AckCmdLine(&i, argc, argv) ) {
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
  _os << "\n\t"; this->numNodes.PrintUsage(_os);
  _os << "\n\t"; this->chunkSize.PrintUsage(_os);
  _os << "\n\t"; this->exactNodes.PrintUsage(_os);
  _os << "\n\t"; gauss_d.PrintUsage(_os);
  
  _os.setf(ios::right,ios::adjustfield);
}


template <class CFG>
void
GaussPRM<CFG>::
PrintValues(ostream& _os){
  _os << "\n" << GetName() << " ";
  _os << this->numNodes.GetFlag() << " " << this->numNodes.GetValue() << " ";
  _os << this->chunkSize.GetFlag() << " " << this->chunkSize.GetValue() << " ";
  _os << this->exactNodes.GetFlag() << " " << this->exactNodes.GetValue() << " ";
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
  out_os << " num nodes = " << this->numNodes.GetValue() << " ";
  out_os << " exact = " << this->exactNodes.GetValue() << " ";
  out_os << " chunk size = " << this->chunkSize.GetValue() << " ";
  out_os << " gauss_d = " << this->gauss_d.GetValue() << " ";
  out_os << endl;
}

template <class CFG>
void
GaussPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      CollisionDetection* cd, DistanceMetric *dm,
	      vector<CFG>& nodes) {
  LOG_DEBUG_MSG("GaussPRM::GenerateNodes()");	

  if (gauss_d.GetValue() == 0)  //if no Gauss_d value given, calculate from robot
    gauss_d.PutValue((_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange());

#ifndef QUIET
  cout << "(numNodes=" << this->numNodes.GetValue() << ") ";
  cout << "(chunkSize=" << this->chunkSize.GetValue() << ") ";
  cout << "(exactNodes=" << this->exactNodes.GetValue() << ") ";
  cout << "(d=" << gauss_d.GetValue() << ") ";
#endif

  CDInfo cdInfo;
  GaussRandomSampler<CFG,true> gauss_sampler(_env, Stats, cd, cdInfo, dm, gauss_d.GetValue());
  int nodes_offset = nodes.size();

  for(int i=0; i<this->numNodes.GetValue(); ++i) {    
    CFG tmp;
    tmp.GetRandomCfg(_env);
    if(this->exactNodes.GetValue() == 1)
      while(!gauss_sampler(tmp, nodes, 1)) {
        tmp.GetRandomCfg(_env);
      }
    else
      gauss_sampler(tmp, nodes, 1);
  }

#if INTERMEDIATE_FILES
  vector<CFG> path;
  copy(nodes.begin()+nodes_offset, nodes.end(),
       back_inserter<vector<CFG> >(path));
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
  CollisionDetection* pCd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric *dm = this->GetMPProblem()->GetDistanceMetric();
 
  GenerateNodes(pEnv,*pStatClass, pCd, dm, outCfgs);
}


#endif
