#ifndef SAMPLER_METHOD_H_
#define SAMPLER_METHOD_H_

#include <iostream>

#ifdef _PARALLEL
#include "runtime.hpp"
#endif

#include "MPLibrary/MPBaseObject.h"
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
/// size_t num, attempts;
/// Boundary* bounds;
/// vector<CfgType> result;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(num, attempts, bounds, back_inserter(result));
/// @endcode
///
/// The other form of @c Sample sends a list of input configurations to apply
/// the sampler rule to.
///
/// @usage
/// @code
/// vector<CfgType> input, result;
/// size_t attempts;
/// Boundary* bounds;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(input.begin(), input.end(), attempts, bounds,
///     back_inserter(result));
/// @endcode
///
/// Both versions of this method offer the option to return the failed attempts
/// with an optional output iterator parameter that is null by default.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
#ifdef _PARALLEL
class SamplerMethod : public MPBaseObject<MPTraits>, public stapl::p_object {
#else
class SamplerMethod : public MPBaseObject<MPTraits> {
#endif

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    SamplerMethod() = default;
    SamplerMethod(XMLNode& _node);
    virtual ~SamplerMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    /// Try to sample a set number of new configurations from a given boundary.
    /// @param[in] _numNodes The number of samples desired.
    /// @param[in] _maxAttempts The maximum number of attempts for each sample.
    /// @param[in] _boundary The boundary to sample from.
    /// @param[out] _result An iterator to storage for the new configurations.
    /// @param[out] _collision An (optional) iterator to storage for failed
    ///                        attempts.
    template <typename ResultOutputIterator,
        typename ColOutputIterator = NullOutputIterator>
    ResultOutputIterator Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, ResultOutputIterator _result,
        ColOutputIterator _collision = ColOutputIterator());

    /// Apply the sampler rule to a set of existing configurations. The output
    /// will generally be a filtered or perturbed version of the input set.
    /// @param[in] _first An iterator to the beginning of a list of input
    ///                   configurations.
    /// @param[in] _last An iterator to the end of a list of input
    ///                  configurations.
    /// @param[in] _maxAttempts The maximum number of attempts to successfully
    ///                         apply the sampler rule to each input.
    /// @param[in] _boundary The sampling boundary to use.
    /// @param[out] _result An iterator to storage for the output configurations.
    /// @param[out] _collision An (optional) iterator to storage for failed
    ///                        attempts.
    template <typename InputIterator, typename ResultOutputIterator,
        typename ColOutputIterator = NullOutputIterator>
    ResultOutputIterator Sample(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        ResultOutputIterator _result,
        ColOutputIterator _collision = ColOutputIterator());

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param[in] _cfg The input configuration.
    /// @param[in] _boundary The sampling boundary.
    /// @param[out] _result The resulting output configurations.
    /// @param[out] _collision The (optional) return for failed attempts.
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) = 0;

    ///@}

    friend class MixSampler<MPTraits>;

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
SamplerMethod<MPTraits>::
SamplerMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
template <typename ResultOutputIterator, typename ColOutputIterator>
ResultOutputIterator
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    ResultOutputIterator _result, ColOutputIterator _collision) {

  Environment* env = this->GetEnvironment();

  for(size_t i = 0; i < _numNodes; ++i) {
    CfgType cfg(this->GetTask()->GetRobot());
    cfg.GetRandomCfg(env, _boundary);
    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _boundary, result, collision))
        break;
    }

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
  }

  return _result;
}


template <typename MPTraits>
template <typename InputIterator, typename ResultOutputIterator,
          typename ColOutputIterator>
ResultOutputIterator
SamplerMethod<MPTraits>::
Sample(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
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

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);
    _first++;
  }

  return _result;
}

/*----------------------------------------------------------------------------*/

#endif
