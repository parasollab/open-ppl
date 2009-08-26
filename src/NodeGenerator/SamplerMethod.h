#ifndef SamplerMethod_h
#define SamplerMethod_h

#include "util.h"
#include <string>
#include <iostream>
#include "Stat_Class.h"
#include "CfgTypes.h"
#include "LabeledObject.h"

class Environment;
class CollisionDetection;
class CDInfo;
class DistanceMetric;
template <class CFG> class Sampler;
template <class CFG, class WEIGHT> class MPRegion;
class MPProblem;
//template<typename CFG> class ValidityChecker;
//////////////////////////////////////////////////////////////////////////////////////////
//
//
//  class SamplerMethod
//  Author @sjacobs May 5, 2009
//
//
//////////////////////////////////////////////////////////////////////////////////////////
/**This is the interface for all node generation methods (prm, obprm, maprm, etc.).
 */
template< typename CFG>
class SamplerMethod : public LabeledObject, public MPBaseObject { 
 private:
 Sampler<CFG>* my_sampler;
 string strLabel;
 
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
  SamplerMethod() { }
  SamplerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem) { 
  strLabel= this->ParseLabelXML( in_Node);
  this->SetLabel(strLabel);
  }
  ///Destructor.	
  virtual ~SamplerMethod() { };

  //@}

 
  Sampler<CFG>* GetSampler() {
  LOG_DEBUG_MSG("SamplerMethod::GetSampler()");
  return my_sampler;
  };

  
  /**Generate nodes according to method type, abstract.
   *@param nodes New created nodes are stored here.
   *the name for this function was: GenerateNodes
   * GenerateNodes has been replaced wiith sample() is no longer needed here, but all classes derived from SamplerMethod must implement the 
   * method for the code to compile
   */
 

 
 
 
};









 



#endif
