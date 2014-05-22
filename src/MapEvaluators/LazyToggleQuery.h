// A mix of Toggle PRM and Lazy PRM

#ifndef LAZYTOGGLEQUERY_H_
#define LAZYTOGGLEQUERY_H_

#include "LazyQuery.h"
#include <deque>

using namespace std;

template<class MPTraits>
class LazyToggleQuery : public LazyQuery<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;

    LazyToggleQuery(const char* _queryFileName = "", string _vcLabel = "") :
      LazyQuery<MPTraits>(_queryFileName, _vcLabel), m_iterative(true) { this->SetName("LazyToggleQuery"); }
    LazyToggleQuery(const CfgType& _start, const CfgType& _goal, string _vcLabel) :
      LazyQuery<MPTraits>(_start, _goal, _vcLabel), m_iterative(true) { this->SetName("LazyToggleQuery"); }
    LazyToggleQuery(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~LazyToggleQuery() {};

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    // Overrides PerformQuery in Query, calls Query::PerformQuery() and adds Toggle functionality
    virtual bool PerformQuery(CfgType _start, CfgType _goal, RoadmapType* _rdmp);

    // Overrides LazyQuery::ProcessInvalidNode, called in LazyQuery::CanRecreatePath()
    virtual void ProcessInvalidNode(const CfgType& node);

  protected:
    string m_toggleConnect;   // Connection method for blocked nodes
    bool m_iterative;         // Process the queue in an iterative fashion?

  private:
    deque<CfgType> m_q;             // A list of both free and blocked witness nodes
};

template<class MPTraits>
LazyToggleQuery<MPTraits>::LazyToggleQuery(MPProblemType* _problem, XMLNodeReader& _node, bool _warn) :
    LazyQuery<MPTraits>(_problem, _node, false) {
  this->SetName("LazyToggleQuery");
  ParseXML(_node);
  if(_warn)
    _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
LazyToggleQuery<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_iterative = _node.boolXMLParameter("iterative", false, true, "Process queue in either iterative or grouped method");
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
    if(citr->getName() == "ToggleConnectionMethod") {
      m_toggleConnect = citr->stringXMLParameter("method", true, "ToggleConnect", "Toggle connection method");
      citr->warnUnrequestedAttributes();
    }
  }
}

template<class MPTraits>
void
LazyToggleQuery<MPTraits>::PrintOptions(ostream& _os) const {
  LazyQuery<MPTraits>::PrintOptions(_os);
  _os << "\ttoggleConnect = " << m_toggleConnect << endl;
  _os << "\titerative = " << m_iterative << endl;
}

// Overrides LazyQuery::ProcessInvalidNode, called in LazyQuery::CanRecreatePath()
template<class MPTraits>
void
LazyToggleQuery<MPTraits>::ProcessInvalidNode(const CfgType& node) {
  if(this->m_debug)
    cout << "*T* Pushing blocked node into queue: " << node << endl;
  m_q.push_back(node);
}

// Overrides PerformQuery in Query, calls Query::PerformQuery() and adds Toggle functionality
template<class MPTraits>
bool
LazyToggleQuery<MPTraits>::PerformQuery(CfgType _start, CfgType _goal, RoadmapType* _rdmp) {

  if(this->m_debug)
    cout << "*T* in LazyToggleQuery::PerformQuery" << endl;

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  RoadmapType* bRdmp = this->GetMPProblem()->GetBlockRoadmap();
  VID sVID = INVALID_VID, gVID = INVALID_VID;

  // If not just started and start/goal aren't connected, go back and sample more
  if(_rdmp->GetGraph()->IsVertex(_start) && _rdmp->GetGraph()->IsVertex(_goal)) {
    sVID = _rdmp->GetGraph()->GetVID(_start);
    gVID = _rdmp->GetGraph()->GetVID(_goal);
    cmap.reset();
    stats->IncGOStat("CC Operations");
    if(!stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID)) {
      if(this->m_debug)
        cout << "*T* After sampling, start and goal still not connected. Sampling again" << endl;
      return false;
    }
  }

  do {
    // Extract paths until start and goal not connected
    if(this->m_debug)
      cout << "*T* Calling Query::PerformQuery" << endl;
    if(Query<MPTraits>::PerformQuery(_start, _goal, _rdmp))
      return true; // Found a path

    if(gVID == INVALID_VID) {
      sVID = _rdmp->GetGraph()->GetVID(_start);
      gVID = _rdmp->GetGraph()->GetVID(_goal);
    }

    // Process the queue
    if(this->m_debug)
      cout << "*T* Processing the queue" << endl;
    while(!m_q.empty()) {
      CfgType node = m_q.front();
      m_q.pop_front();

      if(node.GetLabel("VALID")) { // Lazy-connect
        VID newVID = _rdmp->GetGraph()->AddVertex(node);
        if(this->m_debug)
          cout << "*T* Adding a free node to roadmap: " << node << ", VID = " << newVID << endl;
        for(vector<string>::iterator label = this->m_nodeConnectionLabels.begin();
            label != this->m_nodeConnectionLabels.end(); label++) {
          cmap.reset();
          this->GetMPProblem()->GetConnector(*label)->Connect(_rdmp, *stats, cmap, newVID);
        }
        if(m_iterative) {
          cmap.reset();
          stats->IncGOStat("CC Operations");
          if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID))
            break;
        }
      }

      else { // Toggle-connect
        VID newVID = bRdmp->GetGraph()->AddVertex(node);
        if(this->m_debug)
          cout << "*T* Adding a blocked node to roadmap: " << node << ", VID = " << newVID << endl;
        size_t size = m_q.size();
        this->GetMPProblem()->GetValidityChecker(this->m_vcLabel)->ToggleValidity();
        if(m_iterative)
          this->GetMPProblem()->GetConnector(m_toggleConnect)->Connect(bRdmp, *stats, cmap, newVID, front_inserter(m_q));
        else
          this->GetMPProblem()->GetConnector(m_toggleConnect)->Connect(bRdmp, *stats, cmap, newVID, back_inserter(m_q));
        if(this->m_debug)
          for(size_t i = 0; i < m_q.size()-size; i++)
            cout << "*T* Pushing free node into queue: " << m_q[i] << endl;
        this->GetMPProblem()->GetValidityChecker(this->m_vcLabel)->ToggleValidity();
      }
    }

    // Keep looping if the new lazy-connected nodes put start and goal in same CC
    cmap.reset();
    stats->IncGOStat("CC Operations");
  } while(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID));

  stats->IncGOStat("CC Operations");

  // Start and goal not connected anymore, go back to sampling
  if(this->m_debug)
    cout << "*T* LazyToggleQuery::PerformQuery returning false" << endl;
  return false;
}

#endif

