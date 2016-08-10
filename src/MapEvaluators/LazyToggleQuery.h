#ifndef LAZY_TOGGLE_QUERY_H_
#define LAZY_TOGGLE_QUERY_H_

#include "LazyQuery.h"
#include <deque>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MapEvaluators
/// @brief A mix of Toggle @prm and Lazy @prm
/// @tparam MPTraits Motion planning universe
///
/// TODO.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LazyToggleQuery : public LazyQuery<MPTraits> {

  public:

    ///\name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::MPProblemType    MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType   GraphType;
    typedef typename MPProblemType::VID         VID;

    ///@}
    ///\name Construction
    ///@{

    LazyToggleQuery();
    LazyToggleQuery(MPProblemType* _problem, XMLNode& _node);
    virtual ~LazyToggleQuery() = default;

    ///@}
    ///\name MPBaseObject Overrides
    ///@{

    void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const override;

    ///@}
    ///\name QueryMethod Overrides
    ///@{

    virtual bool PerformSubQuery(const CfgType& _start, const CfgType& _goal)
        override;

    ///@}

  protected:

    ///\name LazyQuery Overrides
    ///@{

    virtual void ProcessInvalidNode(const CfgType& _cfg) override;

    ///@}
    ///\name Helpers
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Process the witness node queue. Iterative version stops when
    ///        a valid node is popped and start and goal are connected.
    /// \param[in] _start The query start.
    /// \param[in] _goal The query goal.
    void ProcessQueue(const VID _start, const VID _goal);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Perform lazy connection of a witness node.
    /// \param[in] _cfg The node to connect.
    void LazyConnect(const CfgType& _cfg);

    ////////////////////////////////////////////////////////////////////////////
    /// \brief Perform toggle connection of a witness node.
    /// \param[in] _cfg The node to connect.
    void ToggleConnect(const CfgType& _cfg);

    ///@}
    ///\name Toggle State
    ///@{

    string m_toggleConnect{"ToggleConnect"}; ///< Connector for blocked nodes.
    deque<CfgType> m_q;       ///< A queue of both free and blocked witness nodes.
    bool m_iterative{true};   ///< Process the queue in an iterative fashion?

    ///@}
    ///\name Unhide QueryMethod names.
    ///@{

    using QueryMethod<MPTraits>::m_path;

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
LazyToggleQuery(MPProblemType* _problem, XMLNode& _node) :
    LazyQuery<MPTraits>(_problem, _node) {
  this->SetName("LazyToggleQuery");
  ParseXML(_node);
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ParseXML(XMLNode& _node) {
  m_iterative = _node.Read("iterative", false, m_iterative,
      "Process queue in either iterative or grouped method");

  for(auto& child : _node)
    if(child.Name() == "ToggleConnectionMethod")
      m_toggleConnect = child.Read("method", true, m_toggleConnect,
          "Toggle connection method");
}


template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
Print(ostream& _os) const {
  LazyQuery<MPTraits>::Print(_os);
  _os << "\ttoggleConnect = " << m_toggleConnect << endl
      << "\titerative = " << m_iterative << endl;
}

/*-------------------------- QueryMethod Overrides ---------------------------*/

template <typename MPTraits>
bool
LazyToggleQuery<MPTraits>::
PerformSubQuery(const CfgType& _start, const CfgType& _goal) {
  VID sVID = INVALID_VID, gVID = INVALID_VID;
  do {
    // Extract paths until a valid one is found or start and goal become
    // disconnected.
    if(LazyQuery<MPTraits>::PerformSubQuery(_start, _goal))
      return true;

    if(gVID == INVALID_VID) {
      auto g = m_path->GetRoadmap()->GetGraph();
      sVID = g->GetVID(_start);
      gVID = g->GetVID(_goal);
    }

    ProcessQueue(sVID, gVID);

    // Keep looping if the new lazy-connected nodes put start and goal in same CC.
  } while(this->SameCC(sVID, gVID));

  // Start and goal not connected anymore, go back to sampling
  if(this->m_debug)
    cout << "\tLazyToggleQuery::PerformQuery returning false." << endl;

  return false;
}

/*--------------------------- LazyQuery Overrides ----------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ProcessInvalidNode(const CfgType& _cfg) {
  if(this->m_debug)
    cout << "\t\tPushing blocked node into queue: " << _cfg << endl;
  m_q.push_back(_cfg);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ProcessQueue(const VID _start, const VID _goal) {
  if(this->m_debug)
    cout << "\tProcessing the toggle queue..." << endl;

  while(!m_q.empty()) {
    CfgType cfg = m_q.front();
    m_q.pop_front();

    if(cfg.GetLabel("VALID")) {
      LazyConnect(cfg);
      if(m_iterative && this->SameCC(_start, _goal))
        break;
    }
    else
      ToggleConnect(cfg);
  }

  if(this->m_debug)
    cout << "\tQueue processing complete." << endl;
}


template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
LazyConnect(const CfgType& _cfg) {
  // Add the configuration to the roadmap.
  auto roadmap = m_path->GetRoadmap();
  VID newVID = roadmap->GetGraph()->AddVertex(_cfg);
  if(this->m_debug)
    cout << "\t\tAdding a free node " << newVID << " to roadmap:\n\t\t"
         << _cfg << endl;

  // Try to connect with each connector.
  for(auto& label : this->m_ncLabels)
    this->GetConnector(label)->Connect(roadmap, newVID);
}


template <typename MPTraits>
void
LazyToggleQuery<MPTraits>::
ToggleConnect(const CfgType& _cfg) {
  auto blockedMap = this->GetMPProblem()->GetBlockRoadmap();
  auto g = blockedMap->GetGraph();

  // Add the new node to the blocked map if not already there.
  if(!g->IsVertex(_cfg))
    return;
  VID newVID = g->AddVertex(_cfg);

  size_t size = m_q.size();
  auto vc = this->GetValidityChecker(this->m_vcLabel);
  auto tc = this->GetConnector(m_toggleConnect);

  vc->ToggleValidity();
  if(m_iterative)
    tc->Connect(blockedMap, newVID, front_inserter(m_q));
  else
    tc->Connect(blockedMap, newVID, back_inserter(m_q));
  vc->ToggleValidity();

  if(this->m_debug) {
    cout << "\t\tAdding a blocked node " << newVID << " to roadmap:\n\t\t  "
         << _cfg << "\n\t\tPushing free nodes into queue:";
    for(size_t i = 0; i < m_q.size() - size; i++)
      cout << "\n\t\t" << i << ") "
           << m_q[m_iterative ? i : m_q.size() - 1 - i];
    cout << endl;
  }
}

/*----------------------------------------------------------------------------*/

#endif
