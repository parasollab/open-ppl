// A mix of Toggle PRM and Lazy PRM

#ifndef LazyToggleQuery_H_
#define LazyToggleQuery_H_

#include "LazyQuery.h"
#include <deque>

using namespace std;

template <class CFG, class WEIGHT>
class LazyToggleQuery : public LazyQuery<CFG, WEIGHT> {

  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;

    LazyToggleQuery(const char* _queryFileName = "", string _vcLabel = "") :
      LazyQuery<CFG, WEIGHT>(_queryFileName, _vcLabel) { this->SetName("LazyToggleQuery"); }
    LazyToggleQuery(CFG _start, CFG _goal, string _vcLabel) :
      LazyQuery<CFG, WEIGHT>(_start, _goal, _vcLabel) { this->SetName("LazyToggleQuery"); }
    LazyToggleQuery(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~LazyToggleQuery() {};

    void ParseXML(XMLNodeReader& _node, bool _warn);
    virtual void PrintOptions(ostream& _os);

    // Overrides PerformQuery in Query, calls Query::PerformQuery() and adds Toggle functionality
    virtual bool PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats);

    // Overrides LazyQuery::ProcessInvalidNode, called in LazyQuery::CanRecreatePath()
    virtual void ProcessInvalidNode(CFG node);

  protected:
    string m_toggleConnect;   // Connection method for blocked nodes
    bool m_iterative;         // Process the queue in an interative fashion?

  private:
    deque<CFG> q;             // A list of both free and blocked witness nodes
};

template <class CFG, class WEIGHT>
LazyToggleQuery<CFG, WEIGHT>::LazyToggleQuery(XMLNodeReader& _node, MPProblem* _problem, bool _warn) :
    LazyQuery<CFG, WEIGHT>(_node, _problem, false) {
  this->SetName("LazyToggleQuery");
  ParseXML(_node, _warn);
  if(this->m_debug)
    PrintOptions(cout);
  if(_warn)
    _node.warnUnrequestedAttributes();
}

template <class CFG, class WEIGHT>
void
LazyToggleQuery<CFG, WEIGHT>::ParseXML(XMLNodeReader& _node, bool _warn) {
  m_iterative = _node.boolXMLParameter("iterative", false, true, "Process queue in either iterative or grouped method");
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
    if(citr->getName() == "ToggleConnectionMethod") {
      m_toggleConnect = citr->stringXMLParameter("method", true, "ToggleConnect", "Toggle connection method");
      citr->warnUnrequestedAttributes();
    }
  }
}

template <class CFG, class WEIGHT>
void
LazyToggleQuery<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  LazyQuery<CFG, WEIGHT>::PrintOptions(_os);
  _os << "\ttoggleConnect = " << m_toggleConnect << endl;
  _os << "\titerative = " << m_iterative << endl;
}

// Overrides LazyQuery::ProcessInvalidNode, called in LazyQuery::CanRecreatePath()
template <class CFG, class WEIGHT>
void
LazyToggleQuery<CFG, WEIGHT>::ProcessInvalidNode(CFG node) {
  if(this->m_debug)
    cout << "*T* Pushing blocked node into queue: " << node << endl;
  q.push_back(node);
}

// Overrides PerformQuery in Query, calls Query::PerformQuery() and adds Toggle functionality
template <class CFG, class WEIGHT>
bool
LazyToggleQuery<CFG, WEIGHT>::PerformQuery(CFG _start, CFG _goal, Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats) {

  if(this->m_debug)
    cout << "*T* in LazyToggleQuery::PerformQuery" << endl;

  StatClass* stats = _rdmp->GetEnvironment()->GetMPProblem()->GetStatClass();
  stapl::sequential::vector_property_map<RoadmapGraph<CFG, WEIGHT>, size_t> cmap;
  Roadmap<CFG, WEIGHT>* bRdmp = this->GetMPProblem()->GetBlockRoadmap();
  VID sVID, gVID = INVALID_VID;

  // If not just started and start/goal aren't connected, go back and sample more
  if(_rdmp->m_pRoadmap->IsVertex(_start)) {
    sVID = _rdmp->m_pRoadmap->GetVID(_start);
    gVID = _rdmp->m_pRoadmap->GetVID(_goal);
    cmap.reset();
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
    if(!stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, sVID, gVID)) {
      if(this->m_debug)
        cout << "*T* After sampling, start and goal still not connected. Sampling again" << endl;
      return false;
    }
  }

  do {
    // Extract paths until start and goal not connected
    if(this->m_debug)
      cout << "*T* Calling Query::PerformQuery" << endl;
    if(Query<CFG, WEIGHT>::PerformQuery(_start, _goal, _rdmp, _stats))
      return true; // Found a path

    vector<VID> freeVIDs, blockVIDs;
    _rdmp->m_pRoadmap->GetVerticesVID(freeVIDs);
    bRdmp->m_pRoadmap->GetVerticesVID(blockVIDs);

    if(gVID == INVALID_VID) {
      sVID = _rdmp->m_pRoadmap->GetVID(_start);
      gVID = _rdmp->m_pRoadmap->GetVID(_goal);
    }

    // Process the queue
    if(this->m_debug)
      cout << "*T* Processing the queue" << endl;
    while(!q.empty()) {
      CFG node = q.front();
      q.pop_front();

      if(node.GetLabel("VALID")) { // Lazy-connect
        vector<VID> newVID(1, _rdmp->m_pRoadmap->AddVertex(node));
        if(this->m_debug)
          cout << "*T* Adding a free node to roadmap: " << node << ", VID = " << newVID[0] << endl;
        for(vector<string>::iterator label = this->m_nodeConnectionLabels.begin();
            label != this->m_nodeConnectionLabels.end(); label++) {
          cmap.reset();
          this->GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*label)->Connect(_rdmp, _stats,
              cmap, newVID.begin(), newVID.end(), freeVIDs.begin(), freeVIDs.end());
        }
        freeVIDs.push_back(newVID[0]);
        if(m_iterative) {
          cmap.reset();
          if(this->m_recordKeep)
            stats->IncGOStat("CC Operations");
          if(stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, sVID, gVID))
            break;
        }
      }

      else { // Toggle-connect
        vector<VID> newVID(1, bRdmp->m_pRoadmap->AddVertex(node));
        if(this->m_debug)
          cout << "*T* Adding a blocked node to roadmap: " << node << ", VID = " << newVID[0] << endl;
        size_t size = q.size();
        this->GetMPProblem()->GetValidityChecker()->GetMethod(this->m_vcLabel)->ToggleValidity();
        if(m_iterative)
          this->GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(m_toggleConnect)->Connect(bRdmp, _stats,
              cmap, newVID.begin(), newVID.end(), blockVIDs.begin(), blockVIDs.end(), front_inserter(q));
        else
          this->GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(m_toggleConnect)->Connect(bRdmp, _stats,
              cmap, newVID.begin(), newVID.end(), blockVIDs.begin(), blockVIDs.end(), back_inserter(q));
        if(this->m_debug)
          for(size_t i = 0; i < q.size()-size; i++)
            cout << "*T* Pushing free node into queue: " << q[i] << endl;
        this->GetMPProblem()->GetValidityChecker()->GetMethod(this->m_vcLabel)->ToggleValidity();
        blockVIDs.push_back(newVID[0]);
      }
    }

    // Keep looping if the new lazy-connected nodes put start and goal in same CC
    cmap.reset();
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
  } while(stapl::sequential::is_same_cc(*(_rdmp->m_pRoadmap), cmap, sVID, gVID));
  if(this->m_recordKeep)
    stats->IncGOStat("CC Operations");

  // Start and goal not connected anymore, go back to sampling
  if(this->m_debug)
    cout << "*T* LazyToggleQuery::PerformQuery returning false" << endl;
  return false;
}

#endif

