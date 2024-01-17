#ifndef PMPL_UNIFORM_RANDOM_SAMPLER_H_
#define PMPL_UNIFORM_RANDOM_SAMPLER_H_

#include "SamplerMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// This sampler only validity-checks the input sample. It is only a
/// 'uniform random' sampler if given a uniform random distribution of input
/// samples.
///
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
class UniformRandomSampler : virtual public SamplerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename SamplerMethod::BoundaryMap;

    ///@}
    ///@name Construction
    ///@{

    UniformRandomSampler();

    UniformRandomSampler(XMLNode& _node);

    virtual ~UniformRandomSampler() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name Sampler Rule
    ///@{

    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        std::vector<Cfg>& _valid, std::vector<Cfg>& _invalid)
        override;

    virtual bool Sampler(GroupCfgType& _cfg, const Boundary* const _boundary,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid)
        override;

    virtual bool Sampler(GroupCfgType& _cfg, const BoundaryMap& _boundaryMap,
        std::vector<GroupCfgType>& _valid, std::vector<GroupCfgType>& _invalid)
        override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_vcLabel;  ///< The validity checker to use.

    ///@}
};

#endif
