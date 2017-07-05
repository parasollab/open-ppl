#ifndef UNIFORM_RANDOM_SAMPLER_H_
#define UNIFORM_RANDOM_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class UniformRandomSampler : public SamplerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;

    ///@}
    ///@name Construction
    ///@{

    UniformRandomSampler(string _vcLabel = "");
    UniformRandomSampler(XMLNode& _node);
    virtual ~UniformRandomSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    string m_vcLabel;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(string _vcLabel) : m_vcLabel(_vcLabel) {
  this->SetName("UniformRandomSampler");
}


template <typename MPTraits>
UniformRandomSampler<MPTraits>::
UniformRandomSampler(XMLNode& _node) : SamplerMethod<MPTraits>(_node) {
  this->SetName("UniformRandomSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
UniformRandomSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
}

/*------------------------------ Sampler Rule --------------------------------*/

template <typename MPTraits>
bool
UniformRandomSampler<MPTraits>::
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {
  // Check containment within environment boundary.
  const bool inBounds = _cfg.InBounds(this->GetEnvironment());

  // Check Validity.
  auto vcm = this->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::SampleImpl()";

  const bool isValid = inBounds and vcm->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _result.push_back(_cfg);
  else
    _collision.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg
              << "\n\tEnvironment boundary: " << *this->GetEnvironment()->GetBoundary()
              << "\n\tBoundary: " << *_boundary
              << "\n\tIn bounds: " << inBounds
              << "\n\tValidity:  " << isValid
              << std::endl;

    VDClearAll();
    VDAddTempCfg(_cfg, isValid);
    if(!inBounds)
      VDComment("UniformSampling::Cfg outside of boundary");
    else
      VDComment("UniformSampling::Cfg " + std::string(isValid ? "" : "in") +
          "valid");
  }

  return isValid;
}

/*----------------------------------------------------------------------------*/

#endif
