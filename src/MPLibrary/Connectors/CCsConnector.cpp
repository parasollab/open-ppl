#include "CCsConnector.h"

#include "MPLibrary/MPLibrary.h"

/*------------------------------- Construction -------------------------------*/

CCsConnector::
CCsConnector() {
  this->SetName("CCsConnector");
}


CCsConnector::
CCsConnector(XMLNode& _node) : ConnectorMethod(_node) {
  this->SetName("CCsConnector");

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

void
CCsConnector::
Print(std::ostream& _os) const {
  ConnectorMethod::Print(_os);
  _os << "\tNeighborhood Finder: " << m_nfLabel
      << std::endl;
}

/*------------------------ ConnectorMethod Overrides -------------------------*/

void
CCsConnector::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  // Find a representative for each CC.
  auto ccTracker = _r->GetCCTracker();
  const VertexSet representatives = ccTracker->GetRepresentatives();

  if(this->m_debug)
    std::cout << "\tThere are " << ccTracker->GetNumCCs() << " CCs."
              << std::endl;

  // Try to connect _source to each CC.
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(this->m_nfLabel);
  const auto& cfg = _r->GetVertex(_source);
  for(const VID rep : representatives) {
    // Skip representatives in the same cc.
    if(ccTracker->InSameCC(_source, rep)) {
      if(this->m_debug)
        std::cout << "\t\tNodes " << _source << ", " << rep << " are already "
                  << "in the same CC."
                  << std::endl;
      continue;
    }

    // Get the representative's CC.
    const VertexSet* const cc = ccTracker->GetCC(rep);
    if(this->m_debug)
      std::cout << "\t\tAttempting connection to CC with node " << rep
                << " of size " << cc->size() << "."
                << std::endl;

    // Determine nearest neighbors in the representative's CC.
    m_neighborBuffer.clear();
    if(_targetSet)
      nf->FindNeighbors(_r, cfg, VertexSetIntersection(*cc, *_targetSet),
          std::back_inserter(m_neighborBuffer));
    else
      nf->FindNeighbors(_r, cfg, *cc, std::back_inserter(m_neighborBuffer));

    // Attempt connections.
    this->ConnectNeighbors(_r, _source, m_neighborBuffer, _collision,
        this->m_skipIfSameCC);
  }

  if(this->m_debug)
    std::cout << "\tThere are now " << ccTracker->GetNumCCs() << " CCs."
              << std::endl;
}
