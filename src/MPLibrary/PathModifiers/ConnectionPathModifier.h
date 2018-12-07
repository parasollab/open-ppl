#ifndef CONNECTION_PATH_MODIFIER_H_
#define CONNECTION_PATH_MODIFIER_H_

#include "Utilities/MPUtils.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "Utilities/SSSP.h"

////////////////////////////////////////////////////////////////////////////////
///
/// A path modifier that uses a connector to connect the given
/// path, then finds a valid path through the connected graph.
///
/// @ingroup PathModifiers
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ConnectionPathModifier : public PathModifierMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Construction
    ///@{

    /// ConnectionPathModifier take the label of the connection method to be
    /// use.
    /// @param _connectionLabel A label to a connection method in the library
    ConnectionPathModifier(const std::string& _connectionLabel);

    ConnectionPathModifier(XMLNode& _node);

    virtual ~ConnectionPathModifier() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}

  protected:

    /// Modifies the input path to a new valid path
    /// @param _graph A graph that contains the path to be modified.
    /// @param _path A path of configurations within a resolution
    ///        distance of each other
    /// @param _newPath An empty vector to place the resulting modified path
    /// @return Success/failed modification
    virtual bool ModifyImpl(RoadmapType* _graph, vector<CfgType>& _path,
        vector<CfgType>& _newPath) override;

  private:

    std::string m_connectionLabel;
};

/*------------------------------ Construction --------------------------------*/
template <typename MPTraits>
ConnectionPathModifier<MPTraits>::
ConnectionPathModifier(const std::string& _connectionLabel = ""):
  PathModifierMethod<MPTraits>(),
  m_connectionLabel(_connectionLabel) {
  this->SetName("ConnectionPathModifier");
}

template <typename MPTraits>
ConnectionPathModifier<MPTraits>::
ConnectionPathModifier(XMLNode& _node) : PathModifierMethod<MPTraits>(_node) {
  this->SetName("ConnectionPathModifier");
  m_connectionLabel = _node.Read("connectionLabel", true, "", "label of connection method");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ConnectionPathModifier<MPTraits>::
Print(ostream& _os) const {
  PathModifierMethod<MPTraits>::Print(_os);
  _os << "\tConnector: " << m_connectionLabel << std::endl;
}

template <typename MPTraits>
bool
ConnectionPathModifier<MPTraits>::
ModifyImpl(RoadmapType* _graph, vector<CfgType>& _path, vector<CfgType>& _newPath) {
  auto graph = _graph ? _graph : this->GetRoadmap();

  auto g = graph;

  auto start = g->GetVID(_path.front());
  auto end = g->GetVID(_path.back());

  auto connector = this->GetConnector(m_connectionLabel);

  connector->Connect(graph);

  SSSPTerminationCriterion<RoadmapType> termination(
      [end](typename RoadmapType::vertex_iterator& _vi,
             const SSSPOutput<RoadmapType>& _sssp) {
        return _vi->descriptor() == end ? SSSPTermination::EndSearch
                                         : SSSPTermination::Continue;
      }
  );

  const SSSPOutput<RoadmapType> sssp = DijkstraSSSP(g, {start}, termination);

  if(!sssp.parent.count(end))
    throw RunTimeException(WHERE) << "Failed to find complete path";

  // Extract the path.
  std::vector<VID> path;
  path.push_back(end);

  VID current = end;
  do {
    current = sssp.parent.at(current);
    path.push_back(current);
  } while(current != start);

  std::for_each(path.rbegin(), path.rend(), [g, &_newPath](VID vid) {
        auto cfg = g->find_vertex(vid)->property();
        _newPath.push_back(cfg);
      });

  return true;
}

#endif
