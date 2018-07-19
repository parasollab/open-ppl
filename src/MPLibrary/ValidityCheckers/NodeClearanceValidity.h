#ifndef NODE_CLEARANCE_VALIDITY_H
#define NODE_CLEARANCE_VALIDITY_H

#include "ValidityCheckerMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ValidityCheckers
/// @brief TODO
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NodeClearanceValidity : public ValidityCheckerMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    NodeClearanceValidity(double _delta = 1.0, std::string _nfLabel = "");
    NodeClearanceValidity(XMLNode& _node);
    virtual ~NodeClearanceValidity();

    virtual void Print(std::ostream& _os) const;

    virtual bool IsValidImpl(CfgType& _cfg, CDInfo& _cdInfo, const std::string& _callName);

  private:
    double m_delta;
    std::string m_nfLabel;
};

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(double _delta, std::string _nfLabel) :
  ValidityCheckerMethod<MPTraits>(), m_delta(_delta), m_nfLabel(_nfLabel) {
    this->SetName("NodeClearanceValidity");
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::NodeClearanceValidity(XMLNode& _node):
  ValidityCheckerMethod<MPTraits>(_node) {
    this->SetName("NodeClearanceValidity");
    m_delta = _node.Read("delta", true, 1.0, 0.0, MAX_DBL, "Clearance from every other node");
    m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder to be used");
  }

template <class MPTraits>
NodeClearanceValidity<MPTraits>::~NodeClearanceValidity() {}

template <class MPTraits>
void
NodeClearanceValidity<MPTraits>::Print(std::ostream& _os) const {
  ValidityCheckerMethod<MPTraits>::Print(_os);
  _os << "\tdelta::" << m_delta
    << "\tnfLabel::" << m_nfLabel << std::endl;
}

template <class MPTraits>
bool
NodeClearanceValidity<MPTraits>::IsValidImpl(CfgType& _cfg,
    CDInfo& _cdInfo, const std::string& _callName) {
  std::vector<Neighbor> kClosest;
  this->GetNeighborhoodFinder(m_nfLabel)->FindNeighbors(
      this->GetRoadmap(), static_cast<CfgType>(_cfg), std::back_inserter(kClosest) );

  if(kClosest.empty()) {
    _cfg.SetLabel("VALID", true);
    return true;
  }

  bool result = m_delta < kClosest[0].distance;

  _cfg.SetLabel("VALID", result);
  return result;
}

#endif
