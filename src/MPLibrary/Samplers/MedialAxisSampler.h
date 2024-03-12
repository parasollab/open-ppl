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
class MedialAxisSampler : public SamplerMethod {

  public:

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    MedialAxisSampler(const MedialAxisUtility& _medialAxisUtility =
        MedialAxisUtility());
    MedialAxisSampler(XMLNode& _node);

    virtual void Print(ostream& _os) const;

    virtual bool Sampler(Cfg& _cfg, const Boundary* const _boundary,
          vector<Cfg>& _result, vector<Cfg>& _collision);

  private:
    MedialAxisUtility m_medialAxisUtility;
};

#endif