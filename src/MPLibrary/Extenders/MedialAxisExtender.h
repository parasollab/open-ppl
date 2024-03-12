#ifndef PMPL_MEDIAL_AXIS_EXTENDER_H_
#define PMPL_MEDIAL_AXIS_EXTENDER_H_

#include "ExtenderMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"


////////////////////////////////////////////////////////////////////////////////
/// Extends along medial axis of @cfree.
///
/// @todo This method needs to have its dedicated MA tool removed and replaced
///       with an MA label (fetch from MPTools).
///
/// Extend along the medial axis of @cfree from \f$q_{near}\f$ towards
/// \f$q_{dir}\f$ until either \f$q_{dir}\f$ is reached, a distance of
/// \f$\Delta q\f$ is extended, or no progress is made.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
class MedialAxisExtender : public ExtenderMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisExtender();

    MedialAxisExtender(XMLNode& _node);

    virtual ~MedialAxisExtender() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name ExtenderMethod Overrides
    ///@{

    virtual bool Extend(const Cfg& _start, const Cfg& _end,
        Cfg& _new, LPOutput& _lp) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MedialAxisUtility m_medialAxisUtility;
                                ///< Tool for pushing Cfgs to the medial axis.

    double m_extendDist;        ///< The step size to use for medial-axis push.
    size_t m_maxIntermediates;  ///< The maximum number of steps to make.
    string m_lpLabel;           ///< The local planner for connecting steps.

    ///@}
};


#endif