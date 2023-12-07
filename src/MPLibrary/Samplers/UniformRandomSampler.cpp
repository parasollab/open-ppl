#include "UniformRandomSampler.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------ Construction --------------------------------*/

UniformRandomSampler::
UniformRandomSampler() {
  this->SetName("UniformRandomSampler");
}


UniformRandomSampler::
UniformRandomSampler(XMLNode& _node) : SamplerMethod(_node) {
  this->SetName("UniformRandomSampler");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
UniformRandomSampler::
Print(std::ostream& _os) const {
  SamplerMethod::Print(_os);
  _os << "\tvcLabel = " << m_vcLabel
      << std::endl;
}

/*------------------------------ Sampler Rule --------------------------------*/

bool
UniformRandomSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid) {
  // Check Validity.
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tBoundary: " << *_boundary
              << "\n\tValidity:  " << isValid
              << std::endl;

    VDClearAll();
    VDAddTempCfg(_cfg, isValid);
    VDComment("UniformSampling::Cfg " + std::string(isValid ? "" : "in") +
        "valid");
  }

  return isValid;
}


bool
UniformRandomSampler::
Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  BoundaryMap emptyMap;
  return Sampler(_cfg, emptyMap, _valid, _invalid);
}


bool
UniformRandomSampler::
Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
    std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid) {
  // Check Validity.
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  const std::string callee = this->GetNameAndLabel() + "::Sampler";
  const bool isValid = vc->IsValid(_cfg, callee);

  // Store result.
  if(isValid)
    _valid.push_back(_cfg);
  else
    _invalid.push_back(_cfg);

  // Debug.
  if(this->m_debug) {
    std::cout << "Sampled Cfg: " << _cfg.PrettyPrint()
              << "\n\tValidity:  " << isValid
              << std::endl;
  }

  return isValid;
}

/*----------------------------------------------------------------------------*/
