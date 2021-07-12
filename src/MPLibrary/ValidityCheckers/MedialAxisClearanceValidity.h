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
template <typename MPTraits>
class MedialAxisClearanceValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MedialAxisClearanceValidity(
        const MedialAxisUtility<MPTraits>& _m = MedialAxisUtility<MPTraits>(),
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

    std::vector<std::pair<CfgType,CfgType>>& GetHistory();
    void ClearHistory();

    ///@}

  protected:

    ///@name ValidityCheckerMethod Overrides
    ///@{

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _callName) override;

    virtual bool IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo,
        const std::string& _caller) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    MedialAxisUtility<MPTraits> m_medialAxisUtility;
    double m_clearance;
    std::vector<std::pair<CfgType,CfgType>> m_history;

    ///@}

};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
MedialAxisClearanceValidity<MPTraits>::
MedialAxisClearanceValidity(const MedialAxisUtility<MPTraits>& _m, double _c)
    : m_medialAxisUtility(_m), m_clearance(_c) {
  this->SetName("MedialAxisClearance");
}

template <typename MPTraits>
MedialAxisClearanceValidity<MPTraits>::
MedialAxisClearanceValidity(XMLNode& _node)
    : ValidityCheckerMethod<MPTraits>(_node),
      m_medialAxisUtility(_node) {
  this->SetName("MedialAxisClearance");

  this->m_clearance = _node.Read("maClearance", true, 0.1,
      0., std::numeric_limits<double>::max(),
      "Medial Axis Validity Clearance");

  m_history.clear();
}

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::
Print(std::ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tMaximum distance from medial axis: " << m_clearance
      << "\tMedialAxisUtility: "
      << std::endl;
  m_medialAxisUtility.Print(_os);
}


/*----------------------------------------------------------------------------*/

template <typename MPTraits>
bool
MedialAxisClearanceValidity<MPTraits>::
IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName) {
  Environment* env = this->GetEnvironment();

  auto vc = this->GetValidityChecker(m_medialAxisUtility.GetValidityCheckerLabel());
  bool isFree = vc->IsValid(_cfg, _cdInfo, _callName);

  if(!isFree){
    _cfg.SetLabel("VALID", !isFree);
    return !isFree;
  }

  CfgType origCfg = _cfg;
  CfgType tmpCfg = _cfg;
  if(!m_medialAxisUtility.PushToMedialAxis(tmpCfg, env->GetBoundary())) {
    _cfg.SetLabel("VALID", false);
    return false;
  }

  m_history.push_back(make_pair(origCfg, tmpCfg));

  const std::string dmLabel = m_medialAxisUtility.GetDistanceMetricLabel();
  const double dist = this->GetDistanceMetric(dmLabel)->Distance(_cfg, tmpCfg);
  const bool result = dist < m_clearance;

  _cfg.SetLabel("VALID", result);

  return result;
}


template <typename MPTraits>
bool
MedialAxisClearanceValidity<MPTraits>::
IsValidImpl(GroupCfgType& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  throw NotImplementedException(WHERE) << "This functions has not been implemented.";
}
/*----------------------------------------------------------------------------*/

template <typename MPTraits>
std::vector< std::pair<typename MPTraits::CfgType, typename MPTraits::CfgType> >&
MedialAxisClearanceValidity<MPTraits>::
GetHistory() {
  return m_history;
}


template <typename MPTraits>
void
MedialAxisClearanceValidity<MPTraits>::
ClearHistory() {
  m_history.clear();
}

/*----------------------------------------------------------------------------*/

#endif
