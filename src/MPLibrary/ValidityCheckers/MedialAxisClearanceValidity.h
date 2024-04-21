#ifndef PMPL_MEDIAL_AXIS_CLEARANCE_VALIDITY_H_
#define PMPL_MEDIAL_AXIS_CLEARANCE_VALIDITY_H_

#include "ValidityCheckerMethod.h"
#include "MPLibrary/MPTools/MedialAxisUtilities.h"


////////////////////////////////////////////////////////////////////////////////
/// Reports configurations as valid iff they are within a threshold distance of
/// the nearest medial axis configuration.
///
/// @todo Remove the history functions.
/// @todo Replace the dedicated MedialAxisUtility with a label and fetch it from
///       MPTools.
/// @ingroup ValidityCheckers
////////////////////////////////////////////////////////////////////////////////
class MedialAxisClearanceValidity : public ValidityCheckerMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisClearanceValidity(
        const MedialAxisUtility& _m = MedialAxisUtility(),
        double _c = 0.001);

    MedialAxisClearanceValidity(XMLNode& _node);

    virtual ~MedialAxisClearanceValidity() {}

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name History
    ///@{
    /// @todo Remove these functions. There is no reason for a validity checker
    ///       to store or report a history of its intermediate computations.
    ///       MedialAxisLP needs to be corrected to handle this.

    std::vector<std::pair<Cfg,Cfg>>& GetHistory();
    void ClearHistory();

    ///@}

  protected:

    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MedialAxisUtility m_medialAxisUtility;
    double m_clearance;
    std::vector<std::pair<Cfg,Cfg>> m_history;

    ///@}

};
#endif