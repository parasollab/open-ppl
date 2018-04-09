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
/// vector<Cfg> result;
/// auto s = this->GetSampler(m_sLabel);
/// s->Sample(num, attempts, bounds, back_inserter(result));
/// @endcode
///
/// The other form of @c Sample sends a list of input configurations to apply
/// the sampler rule to.
///
/// @usage
/// @code
/// vector<Cfg> input, result;
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
class SamplerMethod : public MPBaseObject<MPTraits> {
#else
class MaskedSamplerMethod : public SamplerMethod<MPTraits> {
#endif

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename std::vector<CfgType>::iterator InputIterator;
    typedef typename std::back_insert_iterator<std::vector<CfgType>> OutputIterator;
    typedef typename std::vector<unsigned int> Subassembly;

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
    unsigned int ClearMask();

    /// Accessing functions:
    const std::vector<bool>& GetMask() const { return m_currentMask; };
    void SetMask(const std::vector<bool>& _mask);
    void SetMask(const Cfg& _cfgMask);

    void SetStartCfg(const Cfg& _cfg) { m_startCfg = _cfg; }
    const Cfg& GetStartCfg() { return m_startCfg; }

    //It is the responsibility of the strategy to set this correctly.
    unsigned int GetLastSamplesLeaderBody() { return m_lastSamplesLeaderBody; }

    void SetMaskByBodyList(const std::vector<unsigned int>& _bodyList);

    Subassembly GetActiveBodies() { return m_bodyList; }

    /// Verify that, given the set mask and startCfg, the cfg generated has been
    /// properly created.
    bool VerifyCfg(const Cfg& _input);

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
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        vector<Cfg>& _result, vector<Cfg>& _collision) = 0;

    ///@}

    /// The current mask being used, a false value means the value will be reset
    /// to its m_startCfg value. True means no masking happens for that dof
    /// (marks it as an "active body").
    std::vector<bool> m_currentMask;

    //Body numbers of the masking. Only updated in SetMaskByBodyList().
    std::vector<unsigned int> m_bodyList;//TODO rename to active bodies

    //This is the cfg that has all the default values for dofs that
    // are to be masked. Not formally needed by MaskedRoadmapExtenderSampler.
    Cfg m_startCfg;

    // Since the sampler determines which body is rotated about, this provides
    // access to which body was chosen.
    unsigned int m_lastSamplesLeaderBody;

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
Sample(size_t _numNodes, size_t _maxAttempts, const Boundary* const _boundary,
       OutputIterator _result, OutputIterator _collision) {


  //No sampler should be using this Sample() right now.
  throw RunTimeException(WHERE, "Don't use this sampler, it needs to be "
        "reevaluated (it's a bad choice given the large environments we plan in "
        "for disassembly)");


  // Sample as normal:
  for(size_t i = 0; i < _numNodes; ++i) {
    Cfg cfg(this->GetTask()->GetRobot());

    //If only doing translation, MaskCfgs() handles removing orientation:
    cfg.GetRandomCfg(_boundary);

//    if(m_startCfg != this->GetRoadmap()->GetGraph()->GetVertex(0))
//      throw RunTimeException(WHERE, "startCfg didn't equal root cfg as expected!");

    //TODO: make it so this is scaled to the max extension distance or half it or something.
    cfg /= 30.;//Otherwise rotations will barely show up due to large environments




    //TODO: I should restructure the samplers here! Make cfg always start at the startcfg
    //        then it's up to Sampler() to modify it. Then I can remove ProximitySampler's Sample().






    //Add in the root offset, so the parts aren't on top of each other:
    cfg += m_startCfg;

    vector<Cfg> result;
    vector<Cfg> collision;
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
  vector<Cfg> collision;

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
bool
MaskedSamplerMethod<MPTraits>::
VerifyCfg(const Cfg& _input) {
  /// Note: this is intended to be a debug-only function.
  if(find(m_bodyList.begin(), m_bodyList.end(), m_lastSamplesLeaderBody)
                                               == m_bodyList.end())
    return false;

    //Note the following check is not strictly valid for angular motion:
    const MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
    const unsigned int numBodies = mb->GetNumBodies();
    const unsigned int dofsPerBody = mb->PosDOF() + mb->OrientationDOF();
    const Cfg offsetCfg = _input - m_startCfg;
    for(unsigned int bodyNum = 0; bodyNum < numBodies; ++bodyNum) {
      //If the body is found in m_bodyList, it's validly moving.
      const bool isMovingBody = find(m_bodyList.begin(), m_bodyList.end(),
                                                 bodyNum) != m_bodyList.end();
      for(unsigned int dofNum = 0; dofNum < dofsPerBody; ++dofNum) {
        if(!isMovingBody && abs(offsetCfg[bodyNum*dofsPerBody + dofNum]) > 1e-10) {
          std::cout << "Error reached, the start cfg in the sampler is:"
                    << std::endl << m_startCfg.PrettyPrint() << std::endl
                    << "Offset cfg = " << offsetCfg.PrettyPrint() << std::endl;
          return false;
        }
    }
  }

  std::cout << "Verified cfg = " << _input.PrettyPrint() << std::endl;
  return true;
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

  if(this->m_debug)
    if(!VerifyCfg(_input))
      throw RunTimeException(WHERE, "Generated an improper cfg!");
}


template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
MaskCfgs(std::vector<Cfg>& _input) {
  if(this->m_debug)
    std::cout << "MaskedSamplerMethod::MaskCfgs: Masking cfgs with mask ="
              << std::endl << m_currentMask << std::endl;

  //Mask each cfg in _input, recording whether it was successful or not.
  for(Cfg& sample : _input)
    MaskCfg(sample);
}


template <typename MPTraits>
unsigned int
MaskedSamplerMethod<MPTraits>::
ClearMask() {
  //Return the dofs per body that it sized itself on (note the total size of DOF
  // is number of bodies * dofs per body, we just return dofs per body of this mb)
  const MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  m_currentMask.clear(); // Since resize only sets values for new elements.
  m_currentMask.resize(mb->DOF(), false);

  return (mb->PosDOF() + mb->OrientationDOF());
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
SetMask(const Cfg& _cfgMask) {
  for(bool bit : _cfgMask.GetData())
    m_currentMask  = (bit > 0 ? true : false);//assume >0 is 1, <=0 is a 0.
}

template <typename MPTraits>
void
MaskedSamplerMethod<MPTraits>::
SetMaskByBodyList(const std::vector<unsigned int>& _bodyList) {
  ///@TODO _dofsPerBody isn't strictly needed, consider getting from Task's
  ///      robot's multibody instead.

  // Resize mask based on this Task's robot, returns the dofs per body.
  const unsigned int dofsPerBody = ClearMask();
  m_bodyList = _bodyList;
  for(const unsigned int bodyNum : _bodyList)
    for(std::size_t i = 0; i < dofsPerBody; ++i)
      m_currentMask[bodyNum*dofsPerBody + i] = true;

  if(this->m_debug)
    std::cout << "MaskedSamplerMethod::SetMaskByBodyList: Mask set to "
              << std::endl << m_currentMask << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
