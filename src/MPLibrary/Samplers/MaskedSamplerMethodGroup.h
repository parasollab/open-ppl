#ifndef MASKED_SAMPLER_METHOD_GROUP_H_
#define MASKED_SAMPLER_METHOD_GROUP_H_

#include <iostream>

#include "SamplerMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Base algorithm abstraction for \ref Disassembly Samplers.
///
/// The key addition to the SamplerMethod abstraction is the addition of active
/// robots for group scenarios, along with a "start" configuration which
/// represents the dof values for all of the inactive robots (those not listed
/// in m_activeRobots).
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MaskedSamplerMethodGroup : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType                       GroupCfgType;
    typedef typename std::vector<GroupCfgType>::iterator          InputIterator;
    typedef typename std::back_insert_iterator<std::vector<GroupCfgType>> OutputIterator;
    typedef typename std::vector<size_t>                          Formation;

    ///@}
    ///@name Construction
    ///@{

    MaskedSamplerMethodGroup() = default;
    MaskedSamplerMethodGroup(XMLNode& _node);
    virtual ~MaskedSamplerMethodGroup() = default;

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
    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result,
        OutputIterator _collision)
    { throw RunTimeException(WHERE, "Not Implemented."); }

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result)
    { throw RunTimeException(WHERE, "Not Implemented."); }

    ///@}
    ///@name Sampler Mask Interface
    ///@{

    void SetStartCfg(const GroupCfgType& _cfg) { m_startCfg = _cfg; }

    const GroupCfgType& GetStartCfg() {
      if(!m_startCfg.GetGroupMap())
        throw RunTimeException(WHERE, "Accessing invalid start cfg!");
      return m_startCfg;
    }


    void SetActiveRobots(const Formation& _robots){m_activeRobots = _robots;}

    Formation GetActiveRobots() { return m_activeRobots; }

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    /// Takes a single input configuration and applies the sampler rule to
    /// generate one or more output configurations.
    /// @param[in] _cfg The input configuration, which will be set to m_startCfg
    /// @param[in] _boundary The sampling boundary.
    /// @param[out] _result The resulting output configurations.
    /// @param[out] _collision The (optional) return for failed attempts.
    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        vector<GroupCfgType>& _result, vector<GroupCfgType>& _collision) = 0;

    ///@}

    //Body numbers of the masking. Only updated in SetMaskByBodyList().
    Formation m_activeRobots;

    //This is the cfg that has all the default values for dofs that
    // are to be masked. Not formally needed by MaskedRoadmapExtenderSampler.
    GroupCfgType m_startCfg;
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedSamplerMethodGroup<MPTraits>::
MaskedSamplerMethodGroup(XMLNode& _node) : SamplerMethod<MPTraits>(_node) { }


/*----------------------------------------------------------------------------*/

#endif
