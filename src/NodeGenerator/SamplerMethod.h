#ifndef SamplerMethod_h
#define SamplerMethod_h

#include "util.h"
#include <string>
#include <iostream>
#include "Stat_Class.h"
#include "CfgTypes.h"
#include "LabeledObject.h"
#ifdef _PARALLEL
#include "runtime.h"
#endif

class Environment;
class CollisionDetection;
class CDInfo;
class DistanceMetric;
template <class CFG> class Sampler;
template <class CFG, class WEIGHT> class MPRegion;
class MPProblem;
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
#ifdef _PARALLEL
class SamplerMethod : public LabeledObject, public MPBaseObject,public stapl::p_object { 
#else
class SamplerMethod : public LabeledObject, public MPBaseObject { 
#endif
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
    string strLabel= this->ParseLabelXML( in_Node);
    this->SetLabel(strLabel);
  }
  ///Destructor.	
  virtual ~SamplerMethod() { };

  //@}

  
  /**Generate nodes according to method type, abstract.
   *@param nodes New created nodes are stored here.
   *the name for this function was: GenerateNodes
   * GenerateNodes has been replaced wiith sample() is no longer needed here, but all classes derived from SamplerMethod must implement the 
   * method for the code to compile
   */
 

  virtual const char* name() const = 0; 

  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
  virtual 
  back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stats, 
         typename vector<CFG>::iterator _in_first, typename vector<CFG>::iterator _in_last, int _attempts, 
         back_insert_iterator<vector<CFG> > _out, back_insert_iterator<vector<CFG> > _out_collision) = 0;
  virtual 
  back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stats, 
         int _num_nodes, int _attempts, 
         back_insert_iterator<vector<CFG> > _out, back_insert_iterator<vector<CFG> > _out_collision) = 0;
  virtual 
  back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stats, 
         typename vector<CFG>::iterator _in_first, typename vector<CFG>::iterator _in_last, int _attempts, 
         back_insert_iterator<vector<CFG> > _out) = 0;
  virtual 
  back_insert_iterator<vector<CFG> > 
  Sample(Environment* env, Stat_Class& Stats, 
         int _num_nodes, int _attempts, 
         back_insert_iterator<vector<CFG> > _out) = 0;
 
  //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = typename vector<CFG>::iterator
 public:
  virtual 
  typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stats, 
         typename vector<CFG>::iterator _in_first, typename vector<CFG>::iterator _in_last, int _attempts, 
         typename vector<CFG>::iterator _out, typename vector<CFG>::iterator _out_collision) = 0;
  virtual 
  typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stats, 
         int _num_nodes, int _attempts, 
         typename vector<CFG>::iterator _out, typename vector<CFG>::iterator _out_collision) = 0;
  virtual 
  typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stats, 
         typename vector<CFG>::iterator _in_first, typename vector<CFG>::iterator _in_last, int _attempts, 
         typename vector<CFG>::iterator _out) = 0;
  virtual 
  typename vector<CFG>::iterator 
  Sample(Environment* env, Stat_Class& Stats, 
         int _num_nodes, int _attempts, 
         typename vector<CFG>::iterator _out) = 0;
};

#endif
