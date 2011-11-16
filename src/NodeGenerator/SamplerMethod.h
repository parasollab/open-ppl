#ifndef SamplerMethod_h
#define SamplerMethod_h

#include "util.h"
#include <string>
#include <iostream>
#include "Stat_Class.h"
#include "CfgTypes.h"
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
template <class CFG> class NegateSampler;
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
class SamplerMethod : public MPBaseObject,public stapl::p_object 
#else
class SamplerMethod : public MPBaseObject
#endif
{
  public:
    friend class NegateSampler<CFG>;
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
    SamplerMethod():MPBaseObject() {}
    SamplerMethod(XMLNodeReader& in_Node, MPProblem* in_pProblem):MPBaseObject(in_Node, in_pProblem) {}
    ///Destructor.	
    virtual ~SamplerMethod() { };

    //@}

    /**Generate nodes according to method type, abstract.
     *@param nodes New created nodes are stored here.
     *the name for this function was: GenerateNodes
     * GenerateNodes has been replaced wiith sample() is no longer needed here, but all classes derived from SamplerMethod must implement the 
     * method for the code to compile
     */

    virtual void Print(ostream& os) const { os << this->GetName(); }

    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision) {
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
      }

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          back_insert_iterator<vector<CFG> > result, back_insert_iterator<vector<CFG> > collision) {
        return _Sample(env, Stat, first, last, max_attempts, result, collision);
      }   

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts, 
          back_insert_iterator<vector<CFG> > result) {
        vector<CFG> collision;
        return _Sample(env, Stat, num_nodes, max_attempts, result, back_inserter(collision));
      }

    virtual back_insert_iterator<vector<CFG> > 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          back_insert_iterator<vector<CFG> > result) {
        vector<CFG> collision;
        return _Sample(env, Stat, first, last, max_attempts, result, back_inserter(collision));
      }   

    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision) {
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision);
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          typename vector<CFG>::iterator result, typename vector<CFG>::iterator collision) {
        return _Sample(env, Stat, first, last, max_attempts, result, collision);
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, int num_nodes, int max_attempts,
          typename vector<CFG>::iterator result) {
        vector<CFG> collision(max_attempts * num_nodes);
        return _Sample(env, Stat, num_nodes, max_attempts, result, collision.begin());
      }

    virtual typename vector<CFG>::iterator 
      Sample(Environment* env, Stat_Class& Stat, typename vector<CFG>::iterator first, typename vector<CFG>::iterator last, int max_attempts,
          typename vector<CFG>::iterator result) {
        vector<CFG> collision(max_attempts * distance(first, last));
        return _Sample(env, Stat, first, last, max_attempts, result, collision.begin());
      }
  private:
    template <typename OutputIterator>
      OutputIterator 
      _Sample(Environment* _env, Stat_Class& _stats, int _numNodes, int _maxAttempts, 
          OutputIterator _result, OutputIterator _collision)  
      {
        CFG myCfg;
        vector<CFG> out1;
        CFG collisionOut;
        for (int i =0; i< _numNodes; i++) {
          myCfg.GetRandomCfg(_env);
          while(!this->Sampler(_env, _stats, myCfg, out1, collisionOut, _maxAttempts))
            myCfg.GetRandomCfg(_env);
        }
        _result = copy(out1.begin(), out1.end(), _result);
        *_collision++ = collisionOut;
        return _result;
      }

    template <typename InputIterator, typename OutputIterator>
      OutputIterator 
      _Sample(Environment* _env, Stat_Class& _stats, InputIterator _first, InputIterator _last, int _maxAttempts,
          OutputIterator _result, OutputIterator _collision)  
      {
        while(_first != _last) {
          vector<CFG> resultCfg; 
          CFG collisionCfg;
          if(this->Sampler(_env, _stats, *_first, resultCfg, collisionCfg, _maxAttempts)){
            _result = copy(resultCfg.begin(), resultCfg.end(), _result);
            *_collision++ = collisionCfg;
          }
          _first++;
        }
        return _result;
      }  
  protected:
    virtual bool Sampler(Environment* _env, Stat_Class& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG _cfgCol, int _maxAttempts) = 0;
};

#endif
