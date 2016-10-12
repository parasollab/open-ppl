#ifndef MEDIAL_AXIS_SAMPLERS_H_
#define MEDIAL_AXIS_SAMPLERS_H_

#include "SamplerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MedialAxisSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;

    MedialAxisSampler(const MedialAxisUtility<MPTraits>& _medialAxisUtility =
        MedialAxisUtility<MPTraits>());
    MedialAxisSampler(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
          vector<CfgType>& _result, vector<CfgType>& _collision);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;
};

template <typename MPTraits>
MedialAxisSampler<MPTraits>::
MedialAxisSampler(const MedialAxisUtility<MPTraits>& _medialAxisUtility):
  m_medialAxisUtility(_medialAxisUtility) {
    this->SetName("MedialAxisSampler");
  }

template <typename MPTraits>
MedialAxisSampler<MPTraits>::
MedialAxisSampler(XMLNode& _node):
  SamplerMethod<MPTraits>(_node),
  m_medialAxisUtility(_node) {
    this->SetName("MedialAxisSampler");
  }

template <typename MPTraits>
void
MedialAxisSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  m_medialAxisUtility.Print(_os);
}

template <typename MPTraits>
bool
MedialAxisSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  string call = "MedialAxisSampler::sampler()";
  bool generated = false;

  // If pushed properly and the new CfgType is valid, increment generated
  if(m_medialAxisUtility.PushToMedialAxis(_cfg, _boundary)) {
    generated = true;
    _result.push_back(_cfg);
  }
  return generated;
}

#endif
