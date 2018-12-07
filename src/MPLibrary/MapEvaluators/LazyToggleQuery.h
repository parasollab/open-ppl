#ifndef PMPL_LAZY_TOGGLE_QUERY_H_
#define PMPL_LAZY_TOGGLE_QUERY_H_

#include "LazyQuery.h"

#include <deque>


////////////////////////////////////////////////////////////////////////////////
/// A mix of Toggle @prm and Lazy @prm
///
/// Reference:
///   Jory Denny and Kensen Shi and Nancy Amato. "Lazy Toggle PRM: A
///   Single-Query Approach to Motion Planning". ICRA 2013.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LazyToggleQuery : public LazyQuery<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename MPTraits::GoalTracker  GoalTracker;
    typedef typename GoalTracker::VIDSet    VIDSet;

    ///@}
    ///@name Construction
    ///@{

    LazyToggleQuery();
    LazyToggleQuery(XMLNode& _node);
    virtual ~LazyToggleQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const VID _start, const VIDSet& _goal) override;

    ///@}

  protected:

    ///@name LazyQuery Overrides
    ///@{

    virtual void ProcessInvalidNode(const CfgType& _cfg) override;

    ///@}
    ///@name Helpers
    ///@{

    /// Process the witness node queue. Iterative version stops when a valid node
    /// is popped and start and goal are connected.
    /// @param _start The query start.
    /// @param _goal The query goal.
    void ProcessQueue(const VID _start, const VIDSet& _goal);

    /// Perform lazy connection of a witness node.
    /// @param _cfg The node to connect.
    void LazyConnect(const CfgType& _cfg);

    /// Perform toggle connection of a witness node.
    /// @param _cfg The node to connect.
    void ToggleConnect(const CfgType& _cfg);

    /// Check if a goal node is reachable from some start node, respecting
    /// lazily-invalidated nodes/edges.
    /// @param _start The starting node's descriptor.
    /// @param _goals The goal nodes' descriptors.
    /// @return True if one of the _goals is reachable from _start.
    /// @todo This currently does a partial CC computation on every call. We
    ///       need a more efficient way to handle this.
    bool Reachable(const VID _start, const VIDSet& _goals) const;

    ///@}
    ///@name Toggle State
    ///@{

    std::string m_toggleConnect; ///< Connector for blocked nodes.
    std::deque<CfgType> m_q;     ///< Queue of free and blocked witness nodes.
    bool m_iterative{true};      ///< Process the queue in an iterative fashion?

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
LazyToggleQuery<MPTraits>::
LazyToggleQuery() : LazyQuery<MPTraits>() {
  this->SetName("LazyToggleQuery");
}


template <typename MPTraits>
LazyToggleQuery<MPTraits>::
LazyToggleQuery(XMLNode& _node) : LazyQuery<MPTraits>(_node) {
  this->SetName("LazyToggleQuery");

  m_iterative = _node.Read("iterative", false, m_iterative,
      "Process queue in either iterative or grouped method.");

  for(auto& child : _node)
    // Read up to one 'toggle connect' child node.
    if(child.Name() == "ToggleConnectionMethod" and m_toggleConnect.empty())
      m_toggleConnect = child.Read("label", true, "",
          "Toggle connection method");

  // Assert that we read the toggle connector.
  if(m_toggleConnect.empty())
    throw ParseException(_node.Where()) << "LazyToggleQuery requires a "
                                        << "'ToggleConnectionMethod' child node.";
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
Print(std::ostream& _os) const {
  LazyQuery<MPTraits>::Print(_os);
  _os << "\ttoggleConnect = " << m_toggleConnect
      << "\n\titerative = " << m_iterative
      << std::endl;
}

/*-------------------------- QueryMethod Overrides ---------------------------*/

template <typename MPTraits>
bool
LazyToggleQuery<MPTraits>::
PerformSubQuery(const VID _start, const VIDSet& _goals) {
  do {
    // Extract paths until a valid one is found or start and goal become
    // disconnected.
    if(LazyQuery<MPTraits>::PerformSubQuery(_start, _goals))
      return true;

    ProcessQueue(_start, _goals);

    // Keep looping if the new lazy-connected nodes put start and goal in same CC.
  } while(Reachable(_start, _goals));

  return false;
}

/*--------------------------- LazyQuery Overrides ----------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ProcessInvalidNode(const CfgType& _cfg) {
  if(this->m_debug)
    std::cout << "\t\tPushing blocked node into queue: " << _cfg.PrettyPrint()
              << std::endl;
  m_q.push_back(_cfg);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ProcessQueue(const VID _start, const VIDSet& _goals) {
  if(this->m_debug)
    std::cout << "\tProcessing the toggle queue..." << std::endl;

  while(!m_q.empty()) {
    const CfgType cfg = m_q.front();
    m_q.pop_front();

    if(cfg.GetLabel("VALID")) {
      LazyConnect(cfg);
      if(m_iterative and Reachable(_start, _goals))
        break;
    }
    else
      ToggleConnect(cfg);
  }

  if(this->m_debug)
    std::cout << "\tQueue processing complete." << std::endl;
}


template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
LazyConnect(const CfgType& _cfg) {
  // Add the configuration to the roadmap.
  auto roadmap = this->GetRoadmap();
  const VID newVID = roadmap->AddVertex(_cfg);
  if(this->m_debug)
    std::cout << "\t\tAdding a free node " << newVID << " to roadmap:"
              << "\n\t\t" << _cfg.PrettyPrint()
              << std::endl;

  // Try to connect with each connector.
  for(auto& label : this->m_ncLabels)
    this->GetConnector(label)->Connect(roadmap, newVID);
}


template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ToggleConnect(const CfgType& _cfg) {
  auto blockedMap = this->GetBlockRoadmap();

  // Add the new node to the blocked map if not already there.
  if(!blockedMap->IsVertex(_cfg))
    return;
  const VID newVID = blockedMap->AddVertex(_cfg);

  const size_t size = m_q.size();
  auto vc = this->GetValidityChecker(this->m_vcLabel);
  auto tc = this->GetConnector(m_toggleConnect);

  vc->ToggleValidity();
  if(m_iterative)
    tc->Connect(blockedMap, newVID, std::front_inserter(m_q));
  else
    tc->Connect(blockedMap, newVID, std::back_inserter(m_q));
  vc->ToggleValidity();

  if(this->m_debug) {
    std::cout << "\t\tAdding a blocked node " << newVID << " to roadmap:"
              << "\n\t\t  " << _cfg.PrettyPrint()
              << "\n\t\tPushing free nodes into queue:";
    for(size_t i = 0; i < m_q.size() - size; i++)
      std::cout << "\n\t\t" << i << ") "
                << m_q[m_iterative ? i : m_q.size() - 1 - i].PrettyPrint();
    std::cout << std::endl;
  }
}


template <typename MPTraits>
bool
LazyToggleQuery<MPTraits>::
Reachable(const VID _start, const VIDSet& _goals) const {
  if(this->m_debug)
    cout << "\tChecking connectivity..." << endl;

  auto g = this->GetRoadmap();
  auto stats = this->GetStatClass();

  MethodTimer mt(stats, "LazyToggleQuery::Reachable");
  stats->IncStat("CC Operations");

  // If either the start or goal is unused, these nodes cannot be connected.
  const bool noStart = !this->IsVertexUsed(_start);
  if(noStart) {
    if(this->m_debug)
      std::cout << "\t\tStart " << _start << " marked as unused, cannot connect."
                << std::endl;
    return false;
  }
  bool oneGoodGoal = false;
  for(const VID goal : _goals) {
    oneGoodGoal = this->IsVertexUsed(goal);
    if(oneGoodGoal)
      break;
  }
  if(!oneGoodGoal) {
    if(this->m_debug)
      std::cout << "\t\tGoals " << _goals << " marked as unused, cannot connect."
                << std::endl;
    return false;
  }

  // If the _start is already a _goal, then we are done.
  if(_goals.count(_start)) {
    if(this->m_debug)
      std::cout << "\t\tStart " << _start << " is already a goal."
                << std::endl;
    return true;
  }

  // We cannot use stapl's is_same_cc because it offers no way to ignore
  // specific edges. We will instead do a BFS from _start manually and
  // terminate upon either running out of nodes or finding a goal.
  std::unordered_set<VID> discovered{_start};
  std::queue<VID> queue;
  queue.push(_start);
  VID reachable = INVALID_VID;
  while(reachable == INVALID_VID and !queue.empty()) {
    // Get the next node in the queue.
    const VID current = queue.front();
    queue.pop();

    // Enqueue undiscovered children.
    auto vi = g->find_vertex(current);
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      const VID child = ei->target();

      // Skip if the child is discovered or either child/edge is unused.
      if(discovered.count(child)
          or !this->IsEdgeUsed(ei->id())
          or !this->IsVertexUsed(child))
        continue;

      // If this is the goal, we are done.
      if(_goals.count(child)) {
        reachable = child;
        break;
      }

      // Enqueue the child.
      queue.push(child);
      discovered.insert(child);
    }
  }

  const bool connected = reachable != INVALID_VID;
  if(this->m_debug) {
    if(connected)
      std::cout << "\t\tNodes " << _start << " and " << reachable
                << " are connected."
                << std::endl;
    else
      std::cout << "\t\tStart " << _start << " cannot reach any of the goals "
                << _goals << "."
                << std::endl;
  }

  return connected;
}

/*----------------------------------------------------------------------------*/

#endif
