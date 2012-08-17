// First assumes all nodes and edges are valid,
// then checks for validity in the query phase
// and deletes nodes and edges found to be invalid

#ifndef LazyPRMQuery_H_
#define LazyPRMQuery_H_

#include "Query.h"
#include "StraightLine.h"
#include "ValidityCheckerMethod.hpp"
#include <functional>
#include <algorithm>

using namespace std;

template <class CFG, class WEIGHT>
class LazyPRMQuery : public Query<CFG, WEIGHT> {
  
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    
    LazyPRMQuery(const char* _queryFileName = "", string _vcLabel = "") :
      Query<CFG, WEIGHT>(_queryFileName), m_vcLabel(_vcLabel) { this->SetName("LazyPRMQuery"); }
    LazyPRMQuery(CFG _start, CFG _goal, string _vcLabel) :
      Query<CFG, WEIGHT>(_start, _goal), m_vcLabel(_vcLabel) { this->SetName("LazyPRMQuery"); }
    LazyPRMQuery(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~LazyPRMQuery() {};

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);
   
    // Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
    virtual bool CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
        vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath);

  protected:
    string m_vcLabel;          // Validity checker
    vector<int> m_resolutions; // List of resolution multiples to check
};

template <class CFG, class WEIGHT>
LazyPRMQuery<CFG, WEIGHT>::LazyPRMQuery(XMLNodeReader& _node, MPProblem* _problem) :
    Query<CFG, WEIGHT>(_node, _problem, false) {
  this->SetName("LazyPRMQuery");
  ParseXML(_node);
  if(this->m_debug)
    PrintOptions(cout);
  _node.warnUnrequestedAttributes();
}

template <class CFG, class WEIGHT>
void
LazyPRMQuery<CFG, WEIGHT>::ParseXML(XMLNodeReader& _node) {
  m_vcLabel = _node.stringXMLParameter("vcMethod", false, "default", "Validity checker method"); 
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); citr++) {
    if(citr->getName() == "Resolution") {
      m_resolutions.push_back(citr->numberXMLParameter("mult", true, 1, 1, MAXINT, "Multiple of finest resolution checked"));
      citr->warnUnrequestedAttributes();
    }
  }

  // Sort resolutions in decreasing order, ensure that '1' is included
  if(m_resolutions.empty())
    m_resolutions.push_back(1);
  else {
    sort(m_resolutions.begin(), m_resolutions.end(), greater<int>());
    for(vector<int>::iterator it = m_resolutions.begin(); it+1 != m_resolutions.end(); it++) {
      if(*it == *(it+1))
        it = m_resolutions.erase(it);
    }
    if(m_resolutions[m_resolutions.size()-1] != 1)
      m_resolutions.push_back(1);
  }
}

template <class CFG, class WEIGHT>
void
LazyPRMQuery<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  Query<CFG, WEIGHT>::PrintOptions(_os);
  _os << "\tvc method = " << m_vcLabel;
  _os << "\n\tresolutions =";
  for(typename vector<int>::iterator it = m_resolutions.begin(); it != m_resolutions.end(); it++)
    cout << " " << *it;
  cout << endl;
}

// Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
template <class CFG, class WEIGHT>
bool
LazyPRMQuery<CFG, WEIGHT>::CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
    vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath) {

  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass& stats = *(this->GetMPProblem()->GetMPRegion(0)->GetStatClass());
  string callee = "LazyPRMQuery::CanRecreatePath()";
  CDInfo cdInfo;
  vector<VID> neighbors;
  size_t size = _attemptedPath.size();

  if(this->m_debug) {
    cout << "*** Starting LAZY with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
      << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges\n*** Start vertex checking...";
  }

  // Check vertices for validity
  for(size_t i = 0; i < size; i++) {
    // Check from outside to middle
    size_t index = (i%2 ? size - i/2 - 1 : i/2);
    // Skip checks if already checked and valid
    CFG node = pmpl_detail::GetCfg<RoadmapGraph<CFG, WEIGHT> >()(_rdmp->m_pRoadmap, _attemptedPath[index]);
    if(node.IsLabel("VALID") && node.GetLabel("VALID"))
      continue;
    if(!vc->IsValid(vc->GetVCMethod(m_vcLabel), node, env, stats, cdInfo, true, &callee)) {
      // Delete invalid vertex
      _rdmp->m_pRoadmap->delete_vertex(_attemptedPath[index]);
      if(this->m_debug)
        cout << " FAILED\n*** Ending LAZY with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
          << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges." << endl;
      return false;
    }
  }

  if(this->m_debug)
    cout << " PASSED\n*** Start edge checking...";

  // Check edges for validity
  for(vector<int>::iterator resIt = m_resolutions.begin(); resIt != m_resolutions.end(); resIt++) {
    for(size_t i = 0; i < size-1; i++) {
      // Check from outside to middle
      size_t index = (i%2 ? size - i/2 - 2 : i/2);

      LPOutput<CFG, WEIGHT> ci;
      typename RoadmapGraph<CFG, WEIGHT>::vertex_iterator vi;
      typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator ei;
      typename RoadmapGraph<CFG, WEIGHT>::edge_descriptor ed(_attemptedPath[index], _attemptedPath[index+1]);
      _rdmp->m_pRoadmap->find_edge(ed, vi, ei);

      // Skip checks if already checked and valid
      if((*ei).property().IsChecked(*resIt))
        continue;

      if(this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpLabel)->IsConnected(
            _rdmp->GetEnvironment(), stats, this->GetMPProblem()->GetDistanceMetric()->GetMethod(this->m_dmLabel),
            _rdmp->m_pRoadmap->find_vertex(_attemptedPath[index])->property(),
            _rdmp->m_pRoadmap->find_vertex(_attemptedPath[index+1])->property(), 
            &ci, _rdmp->GetEnvironment()->GetPositionRes() * *resIt,
            _rdmp->GetEnvironment()->GetOrientationRes() * *resIt, true, false))
        (*ei).property().SetChecked(*resIt);
      else {
        // Delete invalid (bidirectional) edge
        _rdmp->m_pRoadmap->delete_edge(_attemptedPath[index], _attemptedPath[index+1]);
        _rdmp->m_pRoadmap->delete_edge(_attemptedPath[index+1], _attemptedPath[index]);

        if(this->m_debug)
          cout << " FAILED\n*** Ending LAZY with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
            << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges." << endl;
        return false;
      }
    }
  }

  if(this->m_debug)
    cout << " PASSED\n*** LAZY: Can recreate path, SUCCESS" << endl;

  // Path all valid, now recreate the path
  return Query<CFG, WEIGHT>::CanRecreatePath(_rdmp, _stats, _attemptedPath, _recreatedPath);

/*  _recreatedPath.push_back(_rdmp->m_pRoadmap->find_vertex(_attemptedPath[0])->property());
  for(typename vector<VID>::iterator it = _attemptedPath.begin(); (it+1) != _attemptedPath.end(); it++) {
    LPOutput<CFG, WEIGHT> lpOut;
    this->GetMPProblem()->GetMPStrategy()->GetLocalPlanners()->GetMethod(this->m_lpLabel)->IsConnected(
        _rdmp->GetEnvironment(), stats, this->GetMPProblem()->GetDistanceMetric()->GetMethod(this->m_dmLabel),
        _rdmp->m_pRoadmap->find_vertex(*it)->property(),
        _rdmp->m_pRoadmap->find_vertex(*(it+1))->property(), 
        &lpOut, _rdmp->GetEnvironment()->GetPositionRes(),
        _rdmp->GetEnvironment()->GetOrientationRes(), false, true, false);
    _recreatedPath.insert(_recreatedPath.end(), lpOut.path.begin(), lpOut.path.end());
    _recreatedPath.push_back(_rdmp->m_pRoadmap->find_vertex(*(it+1))->property());
  }

  if(this->m_debug)
    cout << " PASSED\n*** LAZY: Can recreate path, SUCCESS" << endl;
  return true;*/
}

#endif
