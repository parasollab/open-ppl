#include "RRGStrategy.h"
#include "Connector.h"
#include "MPProblem.h"
#include "MPRegion.h"
#include "MPStrategy.h"


RRGStrategy::RRGStrategy(XMLNodeReader& _node, MPProblem* _problem) :
  BasicRRTStrategy(_node, _problem, false){
    ParseXML(_node);
    _node.warnUnrequestedAttributes();
  }

void 
RRGStrategy::ParseXML(XMLNodeReader& _node) {
  m_nc = _node.stringXMLParameter("connectionMethod",true,"","Node Connection Method");
  if(m_debug) PrintOptions(cout);
}

void 
RRGStrategy::PrintOptions(ostream& _os) {
  BasicRRTStrategy::PrintOptions(_os);
  _os << "\tNode Connection:: " << m_nc << endl;
}

MPStrategyMethod::VID 
RRGStrategy::ExpandTree(int _regionID, CfgType& _dir) {

  VID vid = BasicRRTStrategy::ExpandTree(_regionID, _dir);
  // After expanding, attempt connections to recent node
  if (vid != INVALID_VID) {
    MPRegion<CfgType, WeightType>* region = GetMPProblem()->GetMPRegion(_regionID);
    vector<VID> allVIDs;
    vector<VID> currentVID;
    currentVID.push_back(vid);

    region->GetRoadmap()->m_pRoadmap->GetVerticesVID(allVIDs);
    Connector<CfgType, WeightType>::ConnectionPointer pConnection;
    pConnection = GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(m_nc);    

    // Calling Connect Method and connecting nodes
    stapl::sequential::vector_property_map< GRAPH,size_t > cmap;
    cmap.reset();
    GetMPProblem()->GetMPStrategy()->GetConnector()->Connect(pConnection, region->GetRoadmap(), 
        *(region->GetStatClass()), 
        cmap,
        currentVID.begin(), currentVID.end(), 
        allVIDs.begin(), allVIDs.end());
  }
  return vid;
}
