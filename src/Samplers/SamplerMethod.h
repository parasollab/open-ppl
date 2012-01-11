#ifndef SAMPLERMETHOD_H_
#define SAMPLERMETHOD_H_

#include <string>
#include <iostream>
#include "MetricUtils.h"
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
class SamplerMethod : public MPBaseObject,public stapl::p_object {
#else
class SamplerMethod : public MPBaseObject {
#endif
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
    SamplerMethod() : MPBaseObject() {}
    SamplerMethod(XMLNodeReader& _node, MPProblem* _problem) : MPBaseObject(_node, _problem) {}
    
    ///Destructor.
    virtual ~SamplerMethod() { };

    //@}

    /**Generate nodes according to method type, abstract.
     *@param nodes New created nodes are stored here.
     *the name for this function was: GenerateNodes
     * GenerateNodes has been replaced wiith sample() is no longer needed here, but all classes derived from SamplerMethod must implement the 
     * method for the code to compile
     */

    virtual void PrintOptions(ostream& _out) const { 
      _out << this->GetName() << endl; 
    }

    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = back_insert_iterator<vector<CFG> >
    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts, 
           back_insert_iterator<vector<CFG> > _result, back_insert_iterator<vector<CFG> > _collision) {
      return Sample(_env, _env->GetBoundingBox(), _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb,  StatClass& _stats, int _numNodes, int _maxAttempts, 
           back_insert_iterator<vector<CFG> > _result, back_insert_iterator<vector<CFG> > _collision) {
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CFG> > _result, back_insert_iterator<vector<CFG> > _collision) {
      return Sample(_env, _env->GetBoundingBox(), _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CFG> > _result, back_insert_iterator<vector<CFG> > _collision) {
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision);
    }  

   virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts, 
           back_insert_iterator<vector<CFG> > _result) { 
      return Sample(_env, _env->GetBoundingBox(), _stats, _numNodes, _maxAttempts, _result);
    }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, int _numNodes, int _maxAttempts, 
           back_insert_iterator<vector<CFG> > _result) {
      vector<CFG> _collision;
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, back_inserter(_collision));
    }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CFG> > _result) {
      return Sample(_env, _env->GetBoundingBox(), _stats, _first, _last, _maxAttempts, _result);
     }

    virtual back_insert_iterator<vector<CFG> > 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CFG> > _result) {
      vector<CFG> _collision;
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, back_inserter(_collision));
    }   

    //implementation for InputIterator = vector<CFG>::iterator and OutputIterator = vector<CFG>::iterator
    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CFG>::iterator _result, typename vector<CFG>::iterator _collision) {
      return Sample(_env, _env->GetBoundingBox(), _stats, _numNodes, _maxAttempts, _result, _collision);
    }

   virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CFG>::iterator _result, typename vector<CFG>::iterator _collision) {
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           typename vector<CFG>::iterator _result, typename vector<CFG>::iterator _collision) {
      return Sample(_env, _env->GetBoundingBox(), _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, typename vector<CFG>::iterator _first,
           typename vector<CFG>::iterator _last,   int _maxAttempts,
           typename vector<CFG>::iterator _result, typename vector<CFG>::iterator _collision) {
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CFG>::iterator _result) {      
      return Sample(_env, _env->GetBoundingBox(), _stats, _numNodes, _maxAttempts, _result);
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CFG>::iterator _result) {
      vector<CFG> _collision(_maxAttempts * _numNodes);
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision.begin());
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, StatClass& _stats, typename vector<CFG>::iterator _first, typename vector<CFG>::iterator _last, int _maxAttempts,
           typename vector<CFG>::iterator _result) {      
      return Sample(_env, _env->GetBoundingBox(), _stats, _first, _last, _maxAttempts, _result);
    }

    virtual typename vector<CFG>::iterator 
    Sample(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, typename vector<CFG>::iterator _first,
           typename vector<CFG>::iterator _last, int _maxAttempts,
           typename vector<CFG>::iterator _result) {
      vector<CFG> _collision(_maxAttempts * distance(_first, _last));
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision.begin());
    }

  private:
    template <typename OutputIterator>
    OutputIterator 
    SampleImpl(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts, 
            OutputIterator _result, OutputIterator _collision) {

      return SampleImpl(_env, _env->GetBoundingBox(),  _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    template <typename OutputIterator>
    OutputIterator 
    SampleImpl(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, int _numNodes, int _maxAttempts, 
            OutputIterator _result, OutputIterator _collision) {
      CFG myCfg;
      vector<CFG> out1;
      CFG collisionOut;
      for (int i =0; i< _numNodes; i++) {
        myCfg.GetRandomCfg(_env,_bb);
        while(!this->Sampler(_env, _bb, _stats, myCfg, out1, collisionOut, _maxAttempts))
          myCfg.GetRandomCfg(_env,_bb);
      }
      _result = copy(out1.begin(), out1.end(), _result);
      *_collision++ = collisionOut;
      return _result;
    }

    template <typename InputIterator, typename OutputIterator>
    OutputIterator 
    SampleImpl(Environment* _env,  StatClass& _stats, InputIterator _first, InputIterator _last, int _maxAttempts,
            OutputIterator _result, OutputIterator _collision) {
      return SampleImpl( _env, _env->GetBoundingBox(), _stats, _first, _last, _maxAttempts,
            _result, _collision);

    }

    template <typename InputIterator, typename OutputIterator>
    OutputIterator 
    SampleImpl(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, InputIterator _first, InputIterator _last, int _maxAttempts,
            OutputIterator _result, OutputIterator _collision)  
    {
      while(_first != _last) {
        vector<CFG> resultCfg; 
        CFG collisionCfg;
        if(this->Sampler(_env, _bb, _stats, *_first, resultCfg, collisionCfg, _maxAttempts)){
          _result = copy(resultCfg.begin(), resultCfg.end(), _result);
          *_collision++ = collisionCfg;
        }
        _first++;
      }
      return _result;
    }  
  
  protected:
    virtual bool Sampler(Environment* _env, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) = 0;
    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) = 0;
};

#endif
