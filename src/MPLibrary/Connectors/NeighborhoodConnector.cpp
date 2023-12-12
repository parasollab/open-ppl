#include "NeighborhoodConnector.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

NeighborhoodConnector::
NeighborhoodConnector() {
  this->SetName("NeighborhoodConnector");
}


NeighborhoodConnector::
NeighborhoodConnector(XMLNode& _node) : ConnectorMethod(_node) {
  this->SetName("NeighborhoodConnector");

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

void
NeighborhoodConnector::
Print(std::ostream& _os) const {
  ConnectorMethod::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*------------------------ ConnectorMethod Interface -------------------------*/

void
NeighborhoodConnector::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(this->m_nfLabel);
  const auto& cfg = _r->GetVertex(_source);

  // Determine nearest neighbors.
  m_neighborBuffer.clear();
  if(_targetSet)
    nf->FindNeighbors(_r, cfg, *_targetSet,
        std::back_inserter(m_neighborBuffer));
  else
    nf->FindNeighbors(_r, cfg, std::back_inserter(m_neighborBuffer));

  // Attempt connections.
  this->ConnectNeighbors(_r, _source, m_neighborBuffer, _collision);
}


void
NeighborhoodConnector::
ConnectImpl(GroupRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<GroupRoadmapType>* const _collision) {
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(this->m_nfLabel);
  const auto& cfg = _r->GetVertex(_source);

  // Determine nearest neighbors.
  m_neighborBuffer.clear();
  if(_targetSet)
    nf->FindNeighbors(_r, cfg, *_targetSet,
        std::back_inserter(m_neighborBuffer));
  else
    nf->FindNeighbors(_r, cfg, std::back_inserter(m_neighborBuffer));

  // Attempt connections.
  this->ConnectNeighbors(_r, _source, m_neighborBuffer, _collision);
}

/*----------------------------------------------------------------------------*/
