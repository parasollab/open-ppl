#ifndef PMPL_MASKED_SAMPLER_METHOD_GROUP_H_
#define PMPL_MASKED_SAMPLER_METHOD_GROUP_H_

#include "SamplerMethod.h"
#include "nonstd.h"

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Disassembly Samplers.
///
/// The key addition to the SamplerMethod abstraction is the addition of active
/// robots for group scenarios, along with a "start" configuration which
/// represents the dof values for all of the inactive robots (those not listed
/// in m_activeRobots).
///
/// @todo This method breaks the SamplerMethod API by requiring additional calls
///       for Sample to make sense, and by re-implementing the Filter
///       functionality with a different API. Responsibility for fixing some
///       robots and sampling the others must move to the calling objects
///       because there is no clean way to implement all possibilities with the
///       Sampler API. The Sampler should assume that every robot in the group
///       will be sampled, and that other robots are either marked virtual or in
///       the appropriate position for a collision check. The final fix should
///       sample a configuration for the sub-group of active robots.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MaskedSamplerMethodGroup : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod<MPTraits>::GroupOutputIterator;

    typedef std::vector<size_t> Formation;

    ///@}
    ///@name Construction
    ///@{

    MaskedSamplerMethodGroup() = default;

    MaskedSamplerMethodGroup(XMLNode& _node);

    virtual ~MaskedSamplerMethodGroup() = 0;

    ///@}
    ///@name Sampler Mask Interface
    ///@{
    /// @todo Remove these functions to make the sampler API consistent.

    void SetStartCfg(const GroupCfgType& _cfg);

    const GroupCfgType& GetStartCfg();


    void SetActiveRobots(const Formation& _robots);

    Formation GetActiveRobots();

    ///@}

  protected:

    ///@name Internal State
    ///@{

    /// Body numbers of the masking. Only updated in SetMaskByBodyList().
    Formation m_activeRobots;

    /// This is the cfg that has all the default values for dofs that
    /// are to be masked.
    GroupCfgType m_startCfg;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedSamplerMethodGroup<MPTraits>::
MaskedSamplerMethodGroup(XMLNode& _node) : SamplerMethod<MPTraits>(_node) { }


template <typename MPTraits>
MaskedSamplerMethodGroup<MPTraits>::
~MaskedSamplerMethodGroup() = default;

/*-------------------------- Sampler Mask Interface --------------------------*/

template <typename MPTraits>
void
MaskedSamplerMethodGroup<MPTraits>::
SetStartCfg(const GroupCfgType& _cfg) {
  m_startCfg = _cfg;
}


template <typename MPTraits>
const typename MaskedSamplerMethodGroup<MPTraits>::GroupCfgType&
MaskedSamplerMethodGroup<MPTraits>::
GetStartCfg() {
  if(!m_startCfg.GetGroupRoadmap())
    throw RunTimeException(WHERE, "Accessing invalid start cfg!");
  return m_startCfg;
}


template <typename MPTraits>
void
MaskedSamplerMethodGroup<MPTraits>::
SetActiveRobots(const Formation& _robots) {
  m_activeRobots = _robots;
}


template <typename MPTraits>
typename MaskedSamplerMethodGroup<MPTraits>::Formation
MaskedSamplerMethodGroup<MPTraits>::
GetActiveRobots() {
  return m_activeRobots;
}

/*----------------------------------------------------------------------------*/

#endif
