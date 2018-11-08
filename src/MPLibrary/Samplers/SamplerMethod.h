#ifndef PMPL_SAMPLER_METHOD_H_
#define PMPL_SAMPLER_METHOD_H_

#include <iostream>

#ifdef _PARALLEL
#include "runtime.hpp"
#endif

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

class Boundary;
class Environment;
template <typename MPTraits> class MixSampler;

////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Samplers.
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
/// std::vector<CfgType> valid;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(num, attempts, bounds, std::back_inserter(valid));
/// @endcode
///
/// The other form of @c Sample sends a list of input configurations to apply
/// the sampler rule to.
///
/// @usage
/// @code
/// std::vector<CfgType> input, valid;
/// size_t attempts;
/// Boundary* bounds;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(input.begin(), input.end(), attempts, bounds,
///     std::back_inserter(valid));
/// @endcode
///
/// Both versions of this method offer the option to return the failed attempts
/// with an optional output iterator parameter that is null by default.
/// @ingroup Samplers
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
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;

    typedef typename MPTraits::GroupCfgType GroupCfg;
    typedef typename std::vector<GroupCfg>::iterator GroupInputIterator;
    typedef typename std::back_insert_iterator<std::vector<GroupCfg>> GroupOutputIterator;

    ///@}
    ///@name Construction
    ///@{

    SamplerMethod() = default;
    SamplerMethod(XMLNode& _node);
    virtual ~SamplerMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Sampler Interface
    ///@{

    /// Try to sample a set number of new configurations from a given boundary.
    /// @param _numNodes The number of samples desired.
    /// @param _maxAttempts The maximum number of attempts for each sample.
    /// @param _boundary The boundary to sample from.
    /// @param _valid An iterator to storage for the new configurations.
    /// @param _invalid An (optional) iterator to storage for failed
    ///                   attempts.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _valid,
        OutputIterator _invalid);

    /// \overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _valid);

    /// Try to sample a set number of new configurations from a given boundary
    /// (for the entire robot), and additionally satisfying a constraint
    /// boundary for the end-effector (robot is presumed to have only one EE).
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
        OutputIterator _valid, OutputIterator _invalid);

    /// @overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
        OutputIterator _valid);

    /// Group Cfg versions:
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _valid,
        GroupOutputIterator _invalid);

    /// \overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _valid);

    /// Apply the sampler rule to a set of existing configurations. The output
    /// will generally be a filtered or perturbed version of the input set.
    /// @param _first An iterator to the beginning of a list of input
    ///               configurations.
    /// @param _last An iterator to the end of a list of input
    ///              configurations.
    /// @param _maxAttempts The maximum number of attempts to successfully
    ///                     apply the sampler rule to each input.
    /// @param _boundary The sampling boundary to use.
    /// @param _valid An iterator to storage for the output configurations.
    /// @param _invalid An (optional) iterator to storage for failed
    ///                   attempts.
    virtual void Sample(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        OutputIterator _valid, OutputIterator _invalid);

    /// \overload
    void Sample(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        OutputIterator _valid);


    /// Group Cfg versions:
    virtual void Sample(GroupInputIterator _first, GroupInputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        GroupOutputIterator _valid, GroupOutputIterator _invalid);

    /// \overload
    void Sample(GroupInputIterator _first, GroupInputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        GroupOutputIterator _valid);

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{
    /// The default implementations of these throw exceptions so that derived
    /// classes can avoid implementing unsupported functions.

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param _cfg The input configuration.
    /// @param _boundary The sampling boundary.
    /// @param _valid The resulting output configurations.
    /// @param _invalid The (optional) return for failed attempts.
    /// @return True if a valid configuration was generated, false otherwise.
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid);

    /// This version also specifies a boundary for the end-effector.
    /// @overload
    virtual bool Sampler(CfgType& _cfg, const Boundary* const _robotBoundary,
        const Boundary* const _eeBoundary,
        std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid);

    /// GroupCfg version.
    virtual bool Sampler(GroupCfg& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfg>& _valid, std::vector<GroupCfg>& _invalid);

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
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << std::endl;
}

/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  CfgType cfg(this->GetTask()->GetRobot());
  std::vector<CfgType> valid;
  std::vector<CfgType> invalid;

  for(size_t i = 0; i < _numNodes; ++i) {
    valid.clear();
    invalid.clear();

    // Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      cfg.GetRandomCfg(_boundary);
      stats->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _boundary, valid, invalid))
        break;
    }

    stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
    _valid = copy(valid.begin(), valid.end(), _valid);
    _invalid = copy(invalid.begin(), invalid.end(), _invalid);
  }
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid) {
  std::vector<CfgType> invalid;

  Sample(_numNodes, _maxAttempts, _boundary, _valid, std::back_inserter(invalid));
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  CfgType cfg(this->GetTask()->GetRobot());
  std::vector<CfgType> valid;
  std::vector<CfgType> invalid;

  for(size_t i = 0; i < _numNodes; ++i) {
    valid.clear();
    invalid.clear();

    // Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      cfg.GetRandomCfg(_robotBoundary);
      stats->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _robotBoundary, _eeBoundary, valid, invalid))
        break;
    }

    stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
    _valid = copy(valid.begin(), valid.end(), _valid);
    _invalid = copy(invalid.begin(), invalid.end(), _invalid);
  }
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
    OutputIterator _valid) {
  std::vector<CfgType> invalid;

  Sample(_numNodes, _maxAttempts, _robotBoundary, _eeBoundary, _valid,
      std::back_inserter(invalid));
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  throw RunTimeException(WHERE, "Not implemented!");
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid) {
  throw RunTimeException(WHERE, "Not implemented!");
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid, OutputIterator _invalid) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Sample");

  std::vector<CfgType> valid;
  std::vector<CfgType> invalid;

  while(_first != _last) {
    valid.clear();
    invalid.clear();

    // Terminate when node generated or attempts exhausted.
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      stats->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(*_first, _boundary, valid, invalid))
        break;
    }

    stats->IncNodesGenerated(this->GetNameAndLabel(), valid.size());
    _valid = copy(valid.begin(), valid.end(), _valid);
    _invalid = copy(invalid.begin(), invalid.end(), _invalid);
    _first++;
  }
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(InputIterator _first, InputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    OutputIterator _valid) {
  std::vector<CfgType> invalid;
  Sample(_first, _last, _maxAttempts, _boundary, _valid,
      std::back_inserter(invalid));
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(GroupInputIterator _first, GroupInputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid, GroupOutputIterator _invalid) {
  throw RunTimeException(WHERE, "Not implemented!");
}


template <typename MPTraits>
void
SamplerMethod<MPTraits>::
Sample(GroupInputIterator _first, GroupInputIterator _last,
    size_t _maxAttempts, const Boundary* const _boundary,
    GroupOutputIterator _valid) {
  throw RunTimeException(WHERE, "Not implemented!");
}

/*------------------------------- Sampler Rule -------------------------------*/

template <typename MPTraits>
bool
SamplerMethod<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid)
{
  throw RunTimeException(WHERE) << "No default implementation.";
}


template <typename MPTraits>
bool
SamplerMethod<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _robotBoundary,
    const Boundary* const _eeBoundary,
    std::vector<CfgType>& _valid, std::vector<CfgType>& _invalid)
{
  throw RunTimeException(WHERE) << "No default implementation.";
}


template <typename MPTraits>
bool
SamplerMethod<MPTraits>::
Sampler(GroupCfg& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfg>& _valid, std::vector<GroupCfg>& _invalid) {
  throw RunTimeException(WHERE) << "No default implementation.";
}

/*----------------------------------------------------------------------------*/

#endif
