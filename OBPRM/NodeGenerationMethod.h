#ifndef NodeGenerationMethod_h
#define NodeGenerationMethod_h

#include "Parameters.h"
#include "util.h"

class Environment;
class CollisionDetection;
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
class NodeGenerationMethod { 
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
  ///Destructor.	
  virtual ~NodeGenerationMethod();

  //@}

  //////////////////////
  // Access
  virtual char* GetName() = 0;
  virtual void SetDefault();

  //////////////////////
  // I/O methods
  virtual void ParseCommandLine(int argc, char **argv) = 0;
  virtual void PrintUsage(ostream& _os) = 0;
  virtual void PrintValues(ostream& _os) = 0;
  virtual NodeGenerationMethod<CFG>* CreateCopy() = 0;


  /**Generate nodes according to method type, abstract.
   *@param nodes New created nodes are stored here.
   */
  virtual void GenerateNodes(Environment* _env, Stat_Class& Stats,
                             CollisionDetection* cd, 
			     DistanceMetric *, vector<CFG>& nodes) = 0;

  ///////////////////////////////////////////////////////////////////////////////////////////
  //
  //
  //    Public Data
  //
  //
  //////////////////////////////////////////////////////////////////////////////////////////
  ///Number of nodes to generate.
  num_param<int> numNodes;
  num_param<int> numAttempts;
  CDInfo* cdInfo;
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
  numAttempts      ("attempts",            0,  0,   5000000) {
  numNodes.PutDesc("INTEGER","(number of nodes, default 10)");
  numAttempts.PutDesc("INTEGER","(number of nodes to attempt, default 0 so nodes will be used)");
  SetDefault();
}


template <class CFG>
NodeGenerationMethod<CFG>::
~NodeGenerationMethod() {
}


template <class CFG>
void NodeGenerationMethod<CFG>::
SetDefault() {
  numNodes.PutValue(10);
  numAttempts.PutValue(0);
}

#endif
