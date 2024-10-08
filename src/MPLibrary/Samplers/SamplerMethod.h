#ifndef PMPL_SAMPLER_METHOD_H_
#define PMPL_SAMPLER_METHOD_H_

#ifdef _PARALLEL
#include "runtime.hpp"
#endif

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"

#include <iostream>

class Boundary;
class Environment;

////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Samplers.
///
/// SamplerMethod has three sets of important functions. The first two are the
/// various public methods in the base class, @c Sample and @c Filter, and third
/// is the private virtual function which the derived classes overload,
/// @c Sampler.
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
/// std::vector<Cfg> valid;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(num, attempts, bounds, std::back_inserter(valid));
/// @endcode
///
/// @c Filter sends a list of input configurations to apply the sampler rule to.
///
/// @usage
/// @code
/// std::vector<Cfg> input, valid;
/// size_t attempts;
/// Boundary* bounds;
/// auto s = this->GetSampler(m_sLabel);
/// s->Filter(input.begin(), input.end(), attempts, bounds,
///     std::back_inserter(valid));
/// @endcode
///
/// @todo The present implementation of Sample temporarily saves invalid
///       configurations whether we will use them or not. Avoiding this
///       extraneous retention will likely have good performance benefits in
///       sampling-intensive applications such as proteins and manipulators
///       where the success rate is very low.
///
/// @todo The present layout of this class is confusing since the base method is
///       actually generating uniform random samples which are filtered by the
///       derived class. This does not work for many derived classes as
///       evidenced by their overriding of Sample rather than Sampler: we should
///       rework the design so that derived classes generate and filter their
///       own samples. The base class can then implement its functions in terms
///       of the generate and filter helpers.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
#ifdef _PARALLEL
class SamplerMethod : public MPBaseObject, public stapl::p_object {
#else
class SamplerMethod : public MPBaseObject {
#endif

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    typedef typename std::vector<Cfg>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<Cfg>> OutputIterator;
    typedef typename std::vector<GroupCfgType>::iterator GroupInputIterator;
    typedef typename std::back_insert_iterator<std::vector<GroupCfgType>> GroupOutputIterator;

    /// A map from robots to sampling boundaries.
    typedef std::map<Robot*, const Boundary*> BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    SamplerMethod() = default;

    SamplerMethod(XMLNode& _node);

    virtual ~SamplerMethod() = 0;

    ///@}
    ///@name Individual Configuration Sampling
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
    /// @overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _valid);

    /// Try to sample a set number of new configurations from a given boundary
    /// (for the entire robot), and additionally satisfying a constraint
    /// boundary for the end-effector (robot is presumed to have only one EE).
    /// @param _robotBoundary The boundary for the entire robot.
    /// @param _eeBoundary    The boundary for the end-effector.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
        OutputIterator _valid, OutputIterator _invalid);
    /// This version does not return invalid samples.
    /// @overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _robotBoundary, const Boundary* const _eeBoundary,
        OutputIterator _valid);

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
    virtual void Filter(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        OutputIterator _valid, OutputIterator _invalid);
    /// This version does not return invalid samples.
    /// @overload
    void Filter(InputIterator _first, InputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        OutputIterator _valid);

    ///@}
    ///@name Group Configuration Sampling
    ///@{

    /// Try to sample a set number of new configurations from a single boundary.
    /// @param _numNodes The number of samples desired.
    /// @param _maxAttempts The maximum number of attempts for each sample.
    /// @param _boundary The boundary to sample from.
    /// @param _valid An iterator to storage for the new configurations.
    /// @param _invalid An (optional) iterator to storage for failed
    ///                   attempts.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _valid,
        GroupOutputIterator _invalid);
    ///@example Samplers_UseCase.cpp
    /// This is an example of how to use the sampler methods.

    /// This version does not return invalid samples.
    /// @overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, GroupOutputIterator _valid);

    /// Try to sample a set number of new configurations from a boundary for
    /// each robot.
    /// @param _numNodes The number of samples desired.
    /// @param _maxAttempts The maximum number of attempts for each sample.
    /// @param _boundaryMap A map from robot to sampling boundary. Any robots
    ///                     which are not found in the map will use the
    ///                     environment boundary.
    /// @param _valid An iterator to storage for the new configurations.
    /// @param _invalid An (optional) iterator to storage for failed
    ///                   attempts.
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const BoundaryMap& _boundary, GroupOutputIterator _valid,
        GroupOutputIterator _invalid);
    /// This version does not return invalid samples.
    /// @overload
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const BoundaryMap& _boundary, GroupOutputIterator _valid);

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
    virtual void Filter(GroupInputIterator _first, GroupInputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        GroupOutputIterator _valid, GroupOutputIterator _invalid);
    /// @overload
    void Filter(GroupInputIterator _first, GroupInputIterator _last,
        size_t _maxAttempts, const Boundary* const _boundary,
        GroupOutputIterator _valid);

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{
    /// The default implementations of these throw exceptions so that derived
    /// classes can avoid implementing unsupported functions (as opposed to
    /// making these pure virtual).

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param _cfg The input configuration.
    /// @param _boundary The sampling boundary.
    /// @param _valid The resulting output configurations.
    /// @param _invalid The (optional) return for failed attempts.
    /// @return True if a valid configuration was generated, false otherwise.
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid);

    /// This version also specifies a boundary for the end-effector.
    /// @overload
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _robotBoundary,
        const Boundary* const _eeBoundary,
        std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid);

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param _cfg The input group configuration.
    /// @param _boundary The sampling boundary.
    /// @param _valid The resulting output configurations.
    /// @param _invalid The (optional) return for failed attempts.
    /// @return True if a valid configuration was generated, false otherwise.
    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    /// This version specifies a (possibly different) boundary for each robot.
    /// Robots which are not in the boundary map will use the environment
    /// boundary.
    /// @overload
    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid);

    ///@}

    friend class MixSampler;

};

#endif
