#include "MedialAxisSampler.h"

MedialAxisSampler::
MedialAxisSampler(const MedialAxisUtility& _medialAxisUtility):
  m_medialAxisUtility(_medialAxisUtility) {
    this->SetName("MedialAxisSampler");
  }

MedialAxisSampler::
MedialAxisSampler(XMLNode& _node):
  SamplerMethod(_node),
  m_medialAxisUtility(_node) {
    this->SetName("MedialAxisSampler");
  }

void
MedialAxisSampler::
Print(ostream& _os) const {
  SamplerMethod::Print(_os);
  m_medialAxisUtility.Print(_os);
}

bool
MedialAxisSampler::
Sampler(Cfg& _cfg, const Boundary* const _boundary,
    vector<Cfg>& _result, vector<Cfg>& _collision) {

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