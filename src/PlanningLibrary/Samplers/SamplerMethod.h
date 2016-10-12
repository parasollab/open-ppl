#ifndef SAMPLER_METHOD_H_
#define SAMPLER_METHOD_H_

#include <iostream>

#ifdef _PARALLEL
#include "runtime.hpp"
#endif

#include "PlanningLibrary/MPBaseObject.h"
#include "Utilities/MetricUtils.h"

class Environment;
template <typename MPTraits> class MixSampler;

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
/// SamplerPointer s = this->GetSampler(m_sLabel);
/// size_t num, attempts;
/// shared_ptr<Boundary> bounds;
/// vector<CfgType> result;
/// s->Sample(num, attempts, bounds, back_inserter(result));
/// @endcode
///
/// The other form of @c Sample sends a list of input configurations to apply
/// the sampler rule to.
///
/// @usage
/// @code
/// SamplerPointer s = this->GetSampler(m_sLabel);
/// vector<CfgType> input;
/// size_t attempts;
/// shared_ptr<Boundary> bounds;
/// vector<CfgType> result;
/// s->Sample(input.begin(), input.end(), attempts, bounds, back_inserter(result));
/// @endcode
///
/// There are other versions of this method with the option to return the failed
/// attempts.
///
/// @c Sampler is the private virtual function responsible for taking as input
/// a single input configuration and applying the sampler rule to generate one
/// or more configurations.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
#ifdef _PARALLEL
class SamplerMethod : public MPBaseObject<MPTraits>, public stapl::p_object {
#else
class SamplerMethod : public MPBaseObject<MPTraits> {
#endif
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    SamplerMethod();
    SamplerMethod(XMLNode& _node);
    virtual ~SamplerMethod();

    virtual void Print(ostream& _os) const;

    template<typename ResultOutputIterator,
      typename ColOutputIterator = NullOutputIterator>
        ResultOutputIterator Sample(size_t _numNodes, size_t _maxAttempts,
            shared_ptr<Boundary> _boundary, ResultOutputIterator _result,
            ColOutputIterator _collision = ColOutputIterator());

    template<typename InputIterator, typename ResultOutputIterator,
      typename ColOutputIterator = NullOutputIterator>
        ResultOutputIterator Sample(InputIterator _first, InputIterator _last,
            size_t _maxAttempts, shared_ptr<Boundary> _boundary,
            ResultOutputIterator _result,
            ColOutputIterator _collision = ColOutputIterator());

  protected:
    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) = 0;

    friend class MixSampler<MPTraits>;
};

template <typename MPTraits>
SamplerMethod<MPTraits>::
SamplerMethod() : MPBaseObject<MPTraits>() {
}

template <typename MPTraits>
SamplerMethod<MPTraits>::
SamplerMethod(XMLNode& _node) :
  MPBaseObject<MPTraits>(_node) {
  }

template <typename MPTraits>
SamplerMethod<MPTraits>::
~SamplerMethod() {
};

template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

template <typename MPTraits>
template<typename ResultOutputIterator, typename ColOutputIterator>
ResultOutputIterator
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    shared_ptr<Boundary> _boundary,
    ResultOutputIterator _result, ColOutputIterator _collision) {

  Environment* env = this->GetEnvironment();

  for(size_t i = 0; i < _numNodes; ++i) {
    CfgType cfg;
    cfg.GetRandomCfg(env, _boundary);
    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(), result.size());
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
  }

  return _result;
}

template <typename MPTraits>
template<typename InputIterator, typename ResultOutputIterator, typename ColOutputIterator>
ResultOutputIterator
SamplerMethod<MPTraits>::
Sample(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, shared_ptr<Boundary> _boundary,
    ResultOutputIterator _result, ColOutputIterator _collision) {

  while(_first != _last) {
    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; attempts++) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(*_first, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(), result.size());
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
    _first++;
  }

  return _result;
}

#endif
