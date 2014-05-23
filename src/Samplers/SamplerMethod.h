#ifndef SAMPLERMETHOD_H_
#define SAMPLERMETHOD_H_

#include <string>
#include <iostream>
#include "Utilities/MetricUtils.h"
#ifdef _PARALLEL
#include "runtime.h"
#endif

class Environment;
template<class MPTraits> class MPProblem;
template<class MPTraits> class NegateSampler;
template<class MPTraits> class MixSampler;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Base algorithm abstraction for \ref Samplers.
///
/// SamplerMethod has two sets of important functions. The first are the
/// various public methods in the base class, @c Sample, and second is the
/// private virtual function which the derived classes overload, @c Sampler.
///
/// @c Sample is called in various ways but they break down into two forms:
/// desired number and input configurations. When specifying a desired number
/// @c n of configurations the sampler attempts @c a attempts per desired
/// sample. The output is placed on an output iterator.
///
/// @usage
/// @code
/// SamplerPointer s = this->GetMPProblem()->GetSampler(m_sLabel);
/// int num, attempts;
/// vector<CfgType> result;
/// s->Sample(this->GetMPProblem()->GetEnvironment(),
///           this->GetMPProblem()->GetStatClass(),
///           num, attempts, back_inserter(result));
/// @endcode
///
/// The other form of @c Sample sends a list of input configurations to apply
/// the sampler rule to.
///
/// @usage
/// @code
/// SamplerPointer s = this->GetMPProblem()->GetSampler(m_sLabel);
/// vector<CfgType> input;
/// int attempts;
/// vector<CfgType> result;
/// s->Sample(this->GetMPProblem()->GetEnvironment(),
///           this->GetMPProblem()->GetStatClass(),
///           input.begin(), input.end(), attempts, back_inserter(result));
/// @endcode
///
/// There are other versions of this method with the option to return the failed
/// attempts, specify the boundary used for sampling, etc.
///
/// @c Sampler is the private virtual function responsible for taking as input
/// a single input configuration and applying the sampler rule to generate one
/// or more configurations.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
#ifdef _PARALLEL
class SamplerMethod : public MPBaseObject<MPTraits>, public stapl::p_object {
#else
class SamplerMethod : public MPBaseObject<MPTraits> {
#endif
  public:
    typedef typename MPTraits::CfgType CfgType;

    SamplerMethod() : MPBaseObject<MPTraits>() {}
    SamplerMethod(typename MPTraits::MPProblemType* _problem, XMLNodeReader& _node) : MPBaseObject<MPTraits>(_problem, _node) {}
    virtual ~SamplerMethod() { };

    virtual void Print(ostream& _os) const {
      _os << this->GetNameAndLabel() << endl;
    }

    //implementation for InputIterator = vector<CfgType>::iterator and OutputIterator = back_insert_iterator<vector<CfgType> >
    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result, back_insert_iterator<vector<CfgType> > _collision) {
      return Sample(_env, _env->GetBoundary(), _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, shared_ptr<Boundary> _bb,  StatClass& _stats, int _numNodes, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result, back_insert_iterator<vector<CfgType> > _collision) {
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result, back_insert_iterator<vector<CfgType> > _collision) {
      return Sample(_env, _env->GetBoundary(), _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result, back_insert_iterator<vector<CfgType> > _collision) {
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision);
    }

   virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result) {
      return Sample(_env, _env->GetBoundary(), _stats, _numNodes, _maxAttempts, _result);
    }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, int _numNodes, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result) {
      vector<CfgType> _collision;
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, back_inserter(_collision));
    }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result) {
      return Sample(_env, _env->GetBoundary(), _stats, _first, _last, _maxAttempts, _result);
     }

    virtual back_insert_iterator<vector<CfgType> >
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           back_insert_iterator<vector<CfgType> > _result) {
      vector<CfgType> _collision;
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, back_inserter(_collision));
    }

    //implementation for InputIterator = vector<CfgType>::iterator and OutputIterator = vector<CfgType>::iterator
    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CfgType>::iterator _result, typename vector<CfgType>::iterator _collision) {
      return Sample(_env, _env->GetBoundary(), _stats, _numNodes, _maxAttempts, _result, _collision);
    }

   virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CfgType>::iterator _result, typename vector<CfgType>::iterator _collision) {
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           typename vector<CfgType>::iterator _result, typename vector<CfgType>::iterator _collision) {
      return Sample(_env, _env->GetBoundary(), _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, typename vector<CfgType>::iterator _first,
           typename vector<CfgType>::iterator _last,   int _maxAttempts,
           typename vector<CfgType>::iterator _result, typename vector<CfgType>::iterator _collision) {
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision);
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CfgType>::iterator _result) {
      return Sample(_env, _env->GetBoundary(), _stats, _numNodes, _maxAttempts, _result);
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, int _numNodes, int _maxAttempts,
           typename vector<CfgType>::iterator _result) {
      vector<CfgType> _collision(_maxAttempts * _numNodes);
      return SampleImpl(_env, _bb, _stats, _numNodes, _maxAttempts, _result, _collision.begin());
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, StatClass& _stats, typename vector<CfgType>::iterator _first, typename vector<CfgType>::iterator _last, int _maxAttempts,
           typename vector<CfgType>::iterator _result) {
      return Sample(_env, _env->GetBoundary(), _stats, _first, _last, _maxAttempts, _result);
    }

    virtual typename vector<CfgType>::iterator
    Sample(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, typename vector<CfgType>::iterator _first,
           typename vector<CfgType>::iterator _last, int _maxAttempts,
           typename vector<CfgType>::iterator _result) {
      vector<CfgType> _collision(_maxAttempts * distance(_first, _last));
      return SampleImpl(_env, _bb, _stats, _first, _last, _maxAttempts, _result, _collision.begin());
    }

  private:
    template <typename OutputIterator>
      OutputIterator
      SampleImpl(Environment* _env, shared_ptr<Boundary> _bb, StatClass&
          _stats, int _numNodes, size_t _maxAttempts,
          OutputIterator _result, OutputIterator _collision) {
        CfgType myCfg;
        vector<CfgType> out1;
        vector<CfgType> collisionOut;
        for (int i =0; i< _numNodes; i++) {
          myCfg.GetRandomCfg(_env,_bb);

          //   Terminate when node generated or attempts exhausted
          for(size_t attempts = 0; attempts < _maxAttempts; attempts++){
            if(this->Sampler(_env, _bb, _stats, myCfg, out1, collisionOut))
              break;
            else
              myCfg.GetRandomCfg(_env,_bb);
          }
        }
        _result = copy(out1.begin(), out1.end(), _result);
        _collision = copy(collisionOut.begin(), collisionOut.end(), _collision);
        return _result;
      }

    template <typename InputIterator, typename OutputIterator>
      OutputIterator
      SampleImpl(Environment* _env, shared_ptr<Boundary> _bb, StatClass&
          _stats, InputIterator _first, InputIterator _last, size_t _maxAttempts,
          OutputIterator _result, OutputIterator _collision) {
        while(_first != _last) {
          vector<CfgType> resultCfg;
          vector<CfgType> collisionCfg;
          for(size_t attempts = 0; attempts < _maxAttempts; attempts++){
            if(this->Sampler(_env, _bb, _stats, *_first, resultCfg, collisionCfg)){
              break;
            }
          else{
          }
        }
        _result = copy(resultCfg.begin(), resultCfg.end(), _result);
        _collision = copy(collisionCfg.begin(), collisionCfg.end(), _collision);
        _first++;
      }
      return _result;
    }

  protected:

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
          CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) = 0;

    friend class NegateSampler<MPTraits>;
    friend class MixSampler<MPTraits>;
};

#endif
