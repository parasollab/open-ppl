#ifndef MASKED_SAMPLER_METHOD_H_
#define MASKED_SAMPLER_METHOD_H_

#include <iostream>

#ifdef _PARALLEL
#include "runtime.hpp"
#endif

#include "SamplerMethod.h"
#include "nonstd.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Base algorithm abstraction for \ref Disassembly Samplers.
/// TODO: update this documentation from SamplerMethod
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
class MaskedSamplerMethod : public SamplerMethod<MPTraits> {
#endif

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    MaskedSamplerMethod() = default;
    MaskedSamplerMethod(XMLNode& _node);
    virtual ~MaskedSamplerMethod() = default;

    virtual void Initialize();

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
        OutputIterator _collision) override;

    virtual void Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result) override;

    /// Sample while providing a mask to set and use.
    void SampleMask(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result,
        const std::vector<bool>& _mask);

    // Note there are a couple of other Sample() functions in SamplerMethod that
    // are not worried about in here (yet, at least)

    ///@}
    ///@name Sampler Mask Interface
    ///@{

    /// Function to mask any cfg set.
    /// Public because there might be external uses.
    /// Note that if the vector has cfgs with different DOFs, this will fail.
    void MaskCfgs(std::vector<Cfg>& _input);

    /// Sets mask to default mask (all true values) of the same size.
    void ClearMask();

    /// Accessing functions:
    const std::vector<bool>& GetMask() const;
    void SetMask(const std::vector<bool>& _mask);
    void SetMask(const CfgType& _cfgMask);

    void SetStartCfg(const CfgType& _cfg) { m_startCfg = _cfg; }

    //It is the responsibility of the strategy to set this correctly.
    unsigned int GetLastRotAboutBody() { return m_lastRotAboutBody; }

    bool GetUseOnlyPosDofs() { return m_onlyPositionalDOFs; }
    void SetMaskByBodyList(const std::vector<unsigned int>& _bodyList,
                           const bool _onlyPositionalDOFs = false);
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

    /// The current mask being used, a false value means the value will be reset
    /// to its m_startCfg value. True means no masking happens for that dof
    /// (marks it as an "active body").
    std::vector<bool> m_currentMask;

    //TODO: This should change into getting extracted from the body (so that
    // whatever is set ("translational", "volumetric", "planar", etc...) is used.
    bool m_onlyPositionalDOFs = false;

    //Body numbers of the masking. Only updated in SetMaskByBodyList!
    vector<unsigned int> m_bodyList;

    //This is the cfg that has all the default values for dofs that
    // are to be masked. Not formally needed by MaskedRoadmapExtenderSampler.
    CfgType m_startCfg;

    // Since the sampler determines which body is rotated about, this provides
    // access to which body was chosen.
    unsigned int m_lastRotAboutBody;

    /// Helper function that masks a single cfg. Not public as it's meant to be
    /// called from MaskCfgs().
    void MaskCfg(Cfg& _input);
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedSamplerMethod<MPTraits>::
MaskedSamplerMethod(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {

}


/*---------------------------- Sampler Interface -----------------------------*/

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
Initialize() {
  ClearMask();
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
    const Boundary* const _boundary,
    OutputIterator _result, OutputIterator _collision) {
  // Sample as normal:
  for(size_t i = 0; i < _numNodes; ++i) {
    CfgType cfg(this->GetTask()->GetRobot());

    //If only doing translation, MaskCfgs() handles removing orientation:
    cfg.GetRandomCfg(_boundary);

    if(m_startCfg != this->GetRoadmap()->GetGraph()->GetVertex(0))
      throw RunTimeException(WHERE, "startCfg didn't equal root cfg as expected!");

    //TODO: make it so this is scaled to the max extension distance or half it or something.
    cfg /= 30.;//Otherwise rotations will barely show up due to large environments

    //Add in the root offset, so the parts aren't on top of each other:
    cfg += m_startCfg;

    vector<CfgType> result;
    vector<CfgType> collision;
    //Terminate when node generated or attempts exhausted
    for(size_t attempts = 0; attempts < _maxAttempts; ++attempts) {
      this->GetStatClass()->IncNodesAttempted(this->GetNameAndLabel());
      if(this->Sampler(cfg, _boundary, result, collision))
        break;
    }
    MaskCfgs(result);

    this->GetStatClass()->IncNodesGenerated(this->GetNameAndLabel(),
        result.size());
    _result = copy(result.begin(), result.end(), _result);
    _collision = copy(collision.begin(), collision.end(), _collision);

    if(this->m_debug) {
      std::cout << "MaskedSamplerMethod::Sample result = "
              << nonstd::print_container(result) << std::endl;
    }
  }
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
Sample(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result) {
  vector<CfgType> collision;

  Sample(_numNodes, _maxAttempts, _boundary, _result, back_inserter(collision));
}


template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
SampleMask(size_t _numNodes, size_t _maxAttempts,
        const Boundary* const _boundary, OutputIterator _result,
        const std::vector<bool>& _mask) {
  // Set the mask to what was given:
  SetMask(_mask);

  Sample(_numNodes, _maxAttempts, _boundary, _result);
}


template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
MaskCfg(Cfg& _input) {
  if(m_currentMask.size() != _input.DOF())
    throw RunTimeException(WHERE, "Dimensions don't match up, masking invalid!");

  //Reset masked dofs to their start cfg setting:
  for(size_t i = 0; i < _input.DOF(); i++)
    if(!m_currentMask[i])
      _input[i] = m_startCfg[i];
}


template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
MaskCfgs(std::vector<Cfg>& _input) {
  if(this->m_debug)
    std::cout << "MaskedSamplerMethod::MaskCfgs: Masking cfgs with mask ="
              << std::endl << m_currentMask << std::endl;

  //Mask each cfg in _input, recording whether it was successful or not.
  for(CfgType& sample : _input)
    MaskCfg(sample);
}


template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
ClearMask() {
  CfgType temp(this->GetTask()->GetRobot());
  m_currentMask.clear(); // Since resize only sets values for new elements.
  m_currentMask.resize(temp.DOF(), false);
}

template <typename MPTraits>
const std::vector<bool>&
MaskedSamplerMethod<MPTraits>::
GetMask() const {
  return m_currentMask;
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
SetMask(const std::vector<bool>& _mask) {
  m_currentMask = _mask;
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
SetMask(const CfgType& _cfgMask) {
  std::vector<double> temp;
  for(bool bit : _cfgMask.GetData())
    m_currentMask  = (bit > 0 ? true : false);//assume >0 is 1, <=0 is a 0.
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
SetMaskByBodyList(const std::vector<unsigned int>& _bodyList,
                  const bool _onlyPositionalDOFs) {
  const unsigned int dofsPerBody = 6;
  unsigned int dofsUsedPerBody;
  m_bodyList = _bodyList;
  m_onlyPositionalDOFs = _onlyPositionalDOFs;
  if(_onlyPositionalDOFs)
    dofsUsedPerBody = 3;
  else
    dofsUsedPerBody = 6;

  ClearMask();
  for(auto& bodyNum : _bodyList)
    for(std::size_t i = 0; i < dofsUsedPerBody; i++)
      m_currentMask[bodyNum*dofsPerBody + i] = true;

  if(this->m_debug)
    std::cout << "MaskedSamplerMethod::SetMaskByBodyList: Mask set to "
              << std::endl << m_currentMask << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
