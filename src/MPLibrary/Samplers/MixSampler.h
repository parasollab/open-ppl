#ifndef MIX_SAMPLER_H_
#define MIX_SAMPLER_H_

#include "SamplerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Samples from multiple samplers given a probability distribution.
/// @ingroup Samplers
////////////////////////////////////////////////////////////////////////////////
class MixSampler : virtual public SamplerMethod {

  public:

    MixSampler();
    MixSampler(XMLNode& _node);

    void ParseXML(XMLNode& _node);

    virtual void Print(ostream& _os) const;

  protected:
    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
        vector<Cfg>& _result, vector<Cfg>& _collision);

    std::vector<pair <string, double> > m_samplers; // Stores method label with cumulative probability

    pair<string, int> m_samplerLabels; // Stores label history and index
};

#endif
