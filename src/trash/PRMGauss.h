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
  GaussPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem);
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
  virtual void ParseXML(XMLNodeReader& in_Node);


  //////////////////////
  // I/O methods
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
  double gauss_d; ///< distance, default based on environment
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
GaussPRM() : NodeGenerationMethod<CFG>() {
    SetDefault();
}


template <class CFG>
GaussPRM<CFG>::
~GaussPRM() {
}

template <class CFG>
GaussPRM<CFG>::
GaussPRM(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
NodeGenerationMethod<CFG>(in_Node, in_pProblem) {
  LOG_DEBUG_MSG("GaussPRM::GaussPRM()");
  ParseXML(in_Node);
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
ParseXML(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("GaussPRM::ParseXML()");

  in_Node.verifyName(string("GaussPRM"));
  gauss_d = in_Node.numberXMLParameter(string("gauss_d"),true,double(0.0),
                          double(0.0),double(100000.0),string("gauss_d")); 
  
  PrintOptions(cout);
  LOG_DEBUG_MSG("~GaussPRM::ParseXML()");
}


template <class CFG>
void GaussPRM<CFG>::
SetDefault() {
  NodeGenerationMethod<CFG>::SetDefault();
  gauss_d = double(0.0);
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
  out_os << " num nodes = " << this->numNodes << " ";
  out_os << " exact = " << this->exactNodes << " ";
  out_os << " chunk size = " << this->chunkSize << " ";
  out_os << " gauss_d = " << this->gauss_d << " ";
  out_os << "validity method = " << this->vcMethod << " ";
  out_os << endl;
}

template <class CFG>
void
GaussPRM<CFG>::
GenerateNodes(Environment* _env, Stat_Class& Stats,
	      DistanceMetric *dm, vector<CFG>& nodes) {
  LOG_DEBUG_MSG("GaussPRM::GenerateNodes()");	
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(this->vcMethod);
  if (gauss_d == double(0.0))  //if no Gauss_d value given, calculate from robot
    gauss_d = ((_env->GetMultiBody(_env->GetRobotIndex()))->GetMaxAxisRange());

#ifndef QUIET
  cout << "(numNodes=" << this->numNodes << ") ";
  cout << "(chunkSize=" << this->chunkSize << ") ";
  cout << "(exactNodes=" << this->exactNodes << ") ";
  cout << "(d=" << gauss_d << ") ";
  cout << "(validity method=" << this->vcMethod << ") ";
#endif

  CDInfo cdInfo;
  GaussRandomSampler<CFG,true> gauss_sampler(_env, Stats, vc, vcm, cdInfo, dm, gauss_d);
  int nodes_offset = nodes.size();

  for(int i=0; i<this->numNodes; ++i) {    
    CFG tmp;
    tmp.GetRandomCfg(_env);
    if(this->exactNodes == 1)
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
  //CollisionDetection* pCd = this->GetMPProblem()->GetCollisionDetection();
  DistanceMetric *dm = this->GetMPProblem()->GetDistanceMetric();
  GenerateNodes(pEnv,*pStatClass, dm, outCfgs);
}


#endif
