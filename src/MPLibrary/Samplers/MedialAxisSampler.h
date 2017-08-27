#ifndef MEDIAL_AXIS_SAMPLERS_H_
#define MEDIAL_AXIS_SAMPLERS_H_

#include "SamplerMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"

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

    MedialAxisSampler(const MedialAxisUtility<MPTraits>& _medialAxisUtility =
        MedialAxisUtility<MPTraits>());
    MedialAxisSampler(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, const Boundary* const _boundary,
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
Sampler(CfgType& _cfg, const Boundary* const _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  if(!m_medialAxisUtility.IsInitialized()) {
    m_medialAxisUtility.SetMPLibrary(this->GetMPLibrary());
    m_medialAxisUtility.Initialize();
  }

  if(m_medialAxisUtility.PushToMedialAxis(_cfg, _boundary)) {
    _result.push_back(_cfg);
    return true;
  }
  _collision.push_back(_cfg);
  return false;
}

#endif
