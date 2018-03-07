#ifndef MASKED_RANDOM_SAMPLER_H_
#define MASKED_RANDOM_SAMPLER_H_

#include <iostream>

#include "MaskedSamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Most basic version of the Disassembly Samplers.
///
/// Simply generates a sample in the environment, and is masked based on the set
/// active bodies.
/// @warning There is currently no support for properly sampling rotations for
///          subassemblies yet, so an exception is thrown by default if tried.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MaskedRandomSampler : public MaskedSamplerMethod<MPTraits> {

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

    MaskedRandomSampler();
    MaskedRandomSampler(XMLNode& _node);
    virtual ~MaskedRandomSampler() = default;

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
        vector<CfgType>& _result, vector<CfgType>& _collision);

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
MaskedRandomSampler<MPTraits>::
MaskedRandomSampler(XMLNode& _node) : MaskedSamplerMethod<MPTraits>(_node) {
  this->SetName("MaskedRandomSampler");
}

template <typename MPTraits>
MaskedRandomSampler<MPTraits>::
MaskedRandomSampler() : MaskedSamplerMethod<MPTraits>() {
  this->SetName("MaskedRandomSampler");
}


/*---------------------------- Sampler Interface -----------------------------*/


template <typename MPTraits>
bool
MaskedRandomSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) {

  throw RunTimeException(WHERE, "Don't use this sampler, it needs to be "
      "reevaluated (it's a bad choice given the large environments we plan in "
      "for disassembly)");

  ///@TODO: For now I'm assuming that the bodies all have the same dofsPerBody,
  ///       this should be updated to be arbitrary in the future.
  MultiBody* const mb = this->GetTask()->GetRobot()->GetMultiBody();
  const unsigned int numBodies = mb->GetNumBodies();
  const unsigned int dofsPerBody = mb->DOF() / numBodies;
  const unsigned int posDofsPerBody = mb->PosDOF();
  const unsigned int oriDofsPerBody = mb->OrientationDOF();
  const unsigned int leaderBody = this->m_bodyList[0]; // body rotated about

  // TODO: Move this check:
  // A little bit of sanity checking:
  if((dofsPerBody * numBodies) != mb->DOF() ||
     (posDofsPerBody + oriDofsPerBody) != dofsPerBody)
    throw RunTimeException(WHERE, "DOFs don't match up for multibody!");

  Subassembly bodyList = this->m_bodyList; // Copy so we can rearrange it later

  if (bodyList.size() == 1) { // single part subassembly
    if(this->m_debug)
      std::cout << "rotation params = (";
    for(unsigned int i = posDofsPerBody; i < dofsPerBody; i++) {
      _cfg[leaderBody*dofsPerBody + i] = (DRand() - .5) * 2.; // range: [-1:1]

      if(this->m_debug)
        std::cout << _cfg[leaderBody*dofsPerBody + i] << ", ";
    }
    _cfg.NormalizeOrientation();

    if(this->m_debug) {
      std::cout << ")" << std::endl;
      if(oriDofsPerBody == 3)
        std::cout << "After normalizing = ("
                  << _cfg[leaderBody*dofsPerBody + 3] << ","
                  << _cfg[leaderBody*dofsPerBody + 4] << ","
                  << _cfg[leaderBody*dofsPerBody + 5] << ")" << std::endl;
    }
  }
  else
    throw RunTimeException(WHERE, "Rotations not supported for subassemblies yet!");

  this->m_lastSamplesLeaderBody = bodyList[0];

  //Only check that the sample is in bounds:
  if(!_cfg.InBounds(_boundary)) {
    _collision.push_back(_cfg);
    return false;
  }

  _result.push_back(_cfg);
  return true;
}

/*----------------------------------------------------------------------------*/

#endif
