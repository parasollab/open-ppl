#ifndef NodeGenerationMethod_h
#define NodeGenerationMethod_h

#include "Parameters.h"
#include "util.h"
#include "Stat_Class.h"
#include "CfgTypes.h"

class Environment;
class CollisionDetection;
class CDInfo;
class DistanceMetric;

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
  NodeGenerationMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem);
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
  virtual void ParseXML(TiXmlNode* in_pNode) = 0;
  void ParseXMLnum_nodes(TiXmlNode* in_pNode);

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  ///Used in new MPProblem framework.
  virtual void PrintOptions(ostream& out_os) = 0;
  virtual NodeGenerationMethod<CFG>* CreateCopy() = 0;
  
  //void PrintOptions(ostream& out_os);

  
  /**Generate nodes according to method type, abstract.
   *@param nodes New created nodes are stored here.
   *the name for this function was: GenerateNodes
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
                             CollisionDetection* cd, 
			     DistanceMetric *, vector<CFG>& nodes) = 0;

  ///\todo remove the "{ }" and replace with "= 0" to force all methods 
  virtual void GenerateNodes(vector< pair< CFG, CFGcolorType > > &vectorPair) { };

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///Number of nodes to generate.
  num_param<int> exactNodes;
  num_param<int> numNodes;
  num_param<int> chunkSize;

//  num_param<int> numAttempts;
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
NodeGenerationMethod():  
  numNodes         ("nodes",            10,  1,   5000000),  
  chunkSize        ("chunkSize",        10,  1,   5000000),
  exactNodes ("exact", 0 ,0 ,1)
{
  numNodes.PutDesc("INTEGER","(number of nodes, default 10)");
  chunkSize.PutDesc("INTEGER","(size of chunk, default 10)");
  exactNodes.PutDesc("INTEGER","(whether to generate exact num of nodes, default 0");
  SetDefault();
}

template <class CFG>
NodeGenerationMethod<CFG>::
NodeGenerationMethod(TiXmlNode* in_pNode, MPProblem* in_pProblem) :
     MPBaseObject(in_pNode,in_pProblem) {
}


template <class CFG>
NodeGenerationMethod<CFG>::
~NodeGenerationMethod() {
}


template <class CFG>
void NodeGenerationMethod<CFG>::
SetDefault() {
  numNodes.PutValue(10);
  chunkSize.PutValue(10);
  exactNodes.PutValue(0);
}

template <class CFG>
void NodeGenerationMethod<CFG>::
ParseXMLnum_nodes(TiXmlNode* in_pNode) {
  LOG_DEBUG_MSG("NodeGenerationMethod::ParseXMLnum_nodes()");
  if(!in_pNode) {
    LOG_ERROR_MSG("Error reading <num_nodes> tag....");exit(-1);
  }
  if(string(in_pNode->Value()) != "num_nodes") {
    LOG_ERROR_MSG("Error reading <num_nodes> tag....");exit(-1);
  }

  int nexactNodes, nnumNodes, nchunkSize;
    
  in_pNode->ToElement()->QueryIntAttribute("number",&nnumNodes);
  in_pNode->ToElement()->QueryIntAttribute("exact",&nexactNodes);
  in_pNode->ToElement()->QueryIntAttribute("chunk_size",&nchunkSize);
  exactNodes.SetValue(nexactNodes);
  numNodes.SetValue(nnumNodes);
  chunkSize.SetValue(nchunkSize);
  
  ///\todo remove the comments when everything is ready!
  //cout << "NodeGenerationMethod<CFG>::ParseXMLnum_nodes()" << endl;
  //cout << "   num_nodes  = " << numNodes.GetValue() << endl;
  //cout << "   exact      = " << exactNodes.GetValue() << endl;;
  //cout << "   chunk_size = " << chunkSize.GetValue() << endl;
  LOG_DEBUG_MSG("~NodeGenerationMethod::ParseXMLnum_nodes()");
}
 



#endif
