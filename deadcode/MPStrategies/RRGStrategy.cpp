#include "RRGStrategy.h"
#include "Connector.h"
#include "MPProblem.h"
#include "MPStrategy.h"


RRGStrategy::RRGStrategy(XMLNode& _node, MPProblem* _problem) :
  BasicRRTStrategy(_node, _problem, false){
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  }

void
RRGStrategy::ParseXML(XMLNode& _node) {
  m_nc = _node.Read("connectionMethod",true,"","Node Connection Method");
  if(m_debug) Print(cout);
}

void
RRGStrategy::Print(ostream& _os) const {
  BasicRRTStrategy::Print(_os);
  _os << "\tNode Connection:: " << m_nc << endl;
}

MPStrategyMethod::VID
RRGStrategy::ExpandTree(CfgType& _dir) {

  VID vid = BasicRRTStrategy::ExpandTree(_dir);
  // After expanding, attempt connections to recent node
  if (vid != INVALID_VID) {
    vector<VID> allVIDs;
    vector<VID> currentVID;
    currentVID.push_back(vid);

    GetMPProblem()->GetRoadmap()->m_pRoadmap->GetVerticesVID(allVIDs);
    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(m_nc);

    // Calling Connect Method and connecting nodes
    stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
    pConnection->Connect(GetMPProblem()->GetRoadmap(),
        *(GetMPProblem()->GetStatClass()), cmap,
        currentVID.begin(), currentVID.end(),
        allVIDs.begin(), allVIDs.end());
  }
  return vid;
}
