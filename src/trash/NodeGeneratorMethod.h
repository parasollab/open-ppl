#ifndef NodeGenerationMethod_h
#define NodeGenerationMethod_h

#include "util.h"
#include "Stat_Class.h"
#include "CfgTypes.h"

class Environment;
class CollisionDetection;
class CDInfo;
class DistanceMetric;
template <class CFG, class WEIGHT> class MPRegion;
class MPProblem;
//template<typename CFG> class ValidityChecker;
//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class NodeGenerationMethod
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This is the interface for all node generation methods (prm, obprm, maprm, etc.).
 */
template <class CFG>
class NodeGenerationMethod : public MPBaseObject{ 
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
  NodeGenerationMethod();
  NodeGenerationMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  ///Destructor.	
  virtual ~NodeGenerationMethod();

  //@}

  //////////////////////
  // Access
  virtual char* GetName() = 0;
  virtual void SetDefault();
  virtual int GetNextNodeIndex() = 0;
  virtual void SetNextNodeIndex(int) = 0;
  virtual void IncreaseNextNodeIndex(int) = 0;
  virtual void ParseXML(XMLNodeReader& in_Node) = 0;
  void ParseXMLnum_nodes(XMLNodeReader& in_Node);

  //////////////////////
  // I/O methods
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os) = 0;
  virtual NodeGenerationMethod<CFG>* CreateCopy() = 0;
  
  //void PrintOptions(ostream& out_os);

  
  /**Generate nodes according to method type, abstract.
   *@param nodes New created nodes are stored here.
   *the name for this function was: GenerateNodes
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
			     DistanceMetric *, vector<CFG>& nodes) = 0;

  ///\todo remove the "{ }" and replace with "= 0" to force all methods 
  
  virtual void GenerateNodes(MPRegion<CFG,DefaultWeight>* in_pRegion, vector< CFG > &outCfgs) { };

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///Number of nodes to generate.
  int exactNodes;
  int numNodes;
  int chunkSize;
  std::string vcMethod;
  int m_nExactNodes;
  int m_nNumNodes;
  int m_nMaxCdCalls;
  int m_numCdCalls;
  std::string m_VcMethod;

  CDInfo* cdInfo;
#ifdef COLLISIONCFG
 vector< vector<CFG> >  m_vGeneratedCollisionConfiguration;
#endif
};


/////////////////////////////////////////////////////////////////////
//
//  definitions for class NodeGenerationMethod declarations
//
/////////////////////////////////////////////////////////////////////
template <class CFG>
NodeGenerationMethod<CFG>::
NodeGenerationMethod() {
  SetDefault();
}

template <class CFG>
NodeGenerationMethod<CFG>::
NodeGenerationMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
     MPBaseObject(in_Node,in_pProblem) {
  LOG_DEBUG_MSG("NodeGenerationMethod::NodeGenerationMethod()");

  SetDefault();
  
  XMLNodeReader::childiterator citr;
  for(citr = in_Node.children_begin(); citr!= in_Node.children_end(); ++citr) {
    if(citr->getName() == "num_nodes") {
      ParseXMLnum_nodes(*citr);
    }
  }
  
  LOG_DEBUG_MSG("~NodeGenerationMethod::NodeGenerationMethod()");
}


template <class CFG>
NodeGenerationMethod<CFG>::
~NodeGenerationMethod() {
}


template <class CFG>
void NodeGenerationMethod<CFG>::
SetDefault() {
  numNodes = 10;
  chunkSize = 10;
  exactNodes = 0;
  vcMethod = "cd1";
  LOG_DEBUG_MSG("Validity Checker Method is set to be cd1 by default");
  m_nMaxCdCalls=MAX_INT;
}

template <class CFG>
void NodeGenerationMethod<CFG>::
ParseXMLnum_nodes(XMLNodeReader& in_Node) {
  LOG_DEBUG_MSG("NodeGenerationMethod::ParseXMLnum_nodes()");
  
  in_Node.verifyName(string("num_nodes"));
  

  int nexactNodes, nnumNodes, nMaxCdCalls;
  std::string svc_method;
  nnumNodes = in_Node.numberXMLParameter(string("number"), true, 10,0,MAX_INT,
                                      string("number of nodes"));  
  
  numNodes = nnumNodes;m_nNumNodes = nnumNodes;
  nexactNodes = in_Node.numberXMLParameter(string("exact"), false,1,0,1,
                                      string("exact"));
  exactNodes = nexactNodes;m_nExactNodes = nexactNodes;
  
  svc_method = in_Node.stringXMLParameter(string("vc_method"), false,
                                    string(""), string("Validity Test Method"));
  vcMethod = svc_method;m_VcMethod = svc_method;
  
  nMaxCdCalls = in_Node.numberXMLParameter(string("max_cd_calls"), false,MAX_INT,0,MAX_INT,
                                      string("max num cd calls"));
  m_nMaxCdCalls = nMaxCdCalls;

  
  ///\todo fix this hack, chunkSize should be removed in the future!
  chunkSize = nnumNodes;

  LOG_DEBUG_MSG("~NodeGenerationMethod::ParseXMLnum_nodes()");
}
 



#endif
