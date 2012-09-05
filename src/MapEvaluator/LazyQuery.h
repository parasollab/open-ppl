// First assumes all nodes and edges are valid,
// then checks for validity in the query phase
// and deletes nodes and edges found to be invalid

#ifndef LazyQuery_H_
#define LazyQuery_H_

#include "Query.h"
#include "StraightLine.h"
#include "ValidityCheckerMethod.hpp"
#include <functional>
#include <algorithm>

using namespace std;

template <class CFG, class WEIGHT>
class LazyQuery : public Query<CFG, WEIGHT> {
  
  public:
    typedef typename RoadmapGraph<CFG, WEIGHT>::VID VID;
    
    LazyQuery(const char* _queryFileName = "", string _vcLabel = "") :
      Query<CFG, WEIGHT>(_queryFileName), m_vcLabel(_vcLabel) { this->SetName("LazyQuery"); }
    LazyQuery(CFG _start, CFG _goal, string _vcLabel) :
      Query<CFG, WEIGHT>(_start, _goal), m_vcLabel(_vcLabel) { this->SetName("LazyQuery"); }
    LazyQuery(XMLNodeReader& _node, MPProblem* _problem, bool _warn = true);
    virtual ~LazyQuery() {};

    void ParseXML(XMLNodeReader& _node, bool _warn);
    virtual void PrintOptions(ostream& _os);
   
    // Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
    virtual bool CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
        vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath);

    // Does nothing in LazyQuery; used in LazyToggle, for example
    virtual void ProcessInvalidNode(CFG node) { }

    // Node enhancement step
    virtual void NodeEnhance(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats);

  protected:
    string m_vcLabel;                // Validity checker
    vector<int> m_resolutions;       // List of resolution multiples to check
    int m_numEnhance;                // How many nodes to generate in node enhancement step
    double m_d;                      // Gaussian distance d
    vector<pair<CFG, CFG> > m_edges; // List of removed edges, used in node enhancement
};

template <class CFG, class WEIGHT>
LazyQuery<CFG, WEIGHT>::LazyQuery(XMLNodeReader& _node, MPProblem* _problem, bool _warn) :
    Query<CFG, WEIGHT>(_node, _problem, false) {
  this->SetName("LazyQuery");
  ParseXML(_node, _warn);
  if(_warn)
    _node.warnUnrequestedAttributes();
  if(this->m_debug)
    PrintOptions(cout);
}

template <class CFG, class WEIGHT>
void
LazyQuery<CFG, WEIGHT>::ParseXML(XMLNodeReader& _node, bool _warn) {
  m_vcLabel = _node.stringXMLParameter("vcMethod", false, "default", "Validity checker method");
  m_numEnhance = _node.numberXMLParameter("numEnhance", false, 0, 0, MAX_INT, "Number of nodes to generate in node enhancement");
  m_d = _node.numberXMLParameter("d", false, 0.0, 0.0, MAX_DBL, "Gaussian d value for node enhancement");
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
LazyQuery<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  Query<CFG, WEIGHT>::PrintOptions(_os);
  _os << "\tvc label = " << m_vcLabel;
  _os << "\n\tnumEnhance = " << m_numEnhance;
  _os << "\n\td = " << m_d;
  _os << "\n\tresolutions =";
  for(typename vector<int>::iterator it = m_resolutions.begin(); it != m_resolutions.end(); it++)
    cout << " " << *it;
  cout << endl;
}

// Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
template <class CFG, class WEIGHT>
bool
LazyQuery<CFG, WEIGHT>::CanRecreatePath(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats,
    vector<VID>& _attemptedPath, vector<CFG>& _recreatedPath) {

  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass& stats = *(this->GetMPProblem()->GetStatClass());
  string callee = "LazyQuery::CanRecreatePath()";
  CDInfo cdInfo;
  vector<VID> neighbors;
  size_t size = _attemptedPath.size();

  if(this->m_debug) {
    cout << "*L* Starting LAZY with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
      << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges\n*L* Start vertex checking...\n";
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
      // Add invalid edges to list
      if(m_numEnhance && !node.IsLabel("Enhance")) {
        typename RoadmapGraph<CFG, WEIGHT>::vertex_reference v1 = *(_rdmp->m_pRoadmap->find_vertex(_attemptedPath[index]));
        for(typename RoadmapGraph<CFG, WEIGHT>::adj_edge_iterator aei = v1.begin(); aei != v1.end(); aei++) {
          CFG target = pmpl_detail::GetCfg<RoadmapGraph<CFG, WEIGHT> >()(_rdmp->m_pRoadmap, (*aei).target());
          if(!target.IsLabel("Enhance")) {
            if(this->m_debug)
              cout << "*E* Storing node enhancement edge, VIDs = " << _attemptedPath[index] << ", " << (*aei).target() << endl;
            m_edges.push_back(make_pair(node, target));
          }
        }
      }
      // Delete invalid vertex
      ProcessInvalidNode(node);
      _rdmp->m_pRoadmap->delete_vertex(_attemptedPath[index]);
      if(this->m_debug)
        cout << "*L* Vertex FAILED: " << _attemptedPath[index] << "\n*L* Ending LAZY with: "
          << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
          << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges." << endl;
      return false;
    }
  }

  if(this->m_debug)
    cout << "*L* Vertex checking PASSED, Start edge checking...\n";

  // Check edges for validity
  for(vector<int>::iterator resIt = m_resolutions.begin(); resIt != m_resolutions.end(); resIt++) {
    for(size_t i = 0; i < size-1; i++) {
      // Check from outside to middle
      size_t index = (i%2 ? size - i/2 - 2 : i/2);

      CFG witness;
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
            witness, &ci, _rdmp->GetEnvironment()->GetPositionRes() * *resIt,
            _rdmp->GetEnvironment()->GetOrientationRes() * *resIt, true, false))
        (*ei).property().SetChecked(*resIt);
      else {
        // Add invalid edge to list
        if(m_numEnhance) {
          CFG cfg1 = pmpl_detail::GetCfg<RoadmapGraph<CFG, WEIGHT> >()(_rdmp->m_pRoadmap, _attemptedPath[index]);
          CFG cfg2 = pmpl_detail::GetCfg<RoadmapGraph<CFG, WEIGHT> >()(_rdmp->m_pRoadmap, _attemptedPath[index+1]);
          if(!cfg1.IsLabel("Enhance") && !cfg2.IsLabel("Enhance")) {
            if(this->m_debug)
              cout << "*E* Storing node enhancement edge, VIDs = " << _attemptedPath[index] << ", " <<
                _attemptedPath[index+1] << endl;
            m_edges.push_back(make_pair(cfg1, cfg2));
          }
        }
        // Delete invalid (bidirectional) edge
        ProcessInvalidNode(witness);
        _rdmp->m_pRoadmap->delete_edge(_attemptedPath[index], _attemptedPath[index+1]);
        _rdmp->m_pRoadmap->delete_edge(_attemptedPath[index+1], _attemptedPath[index]);
        if(this->m_debug)
          cout << "*L* Edge FAILED: " << _attemptedPath[index] << ", " << _attemptedPath[index+1]
            << "\n*L* Ending LAZY with: " << _rdmp->m_pRoadmap->get_num_vertices() << " vertices, "
            << _rdmp->m_pRoadmap->get_num_edges()/2 << " edges." << endl;
        return false;
      }
    }
  }

  // Path all valid, now recreate the path
  _recreatedPath.push_back(_rdmp->m_pRoadmap->find_vertex(_attemptedPath[0])->property());
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
    cout << "*L* Edge checking PASSED\n*L* LAZY: Can recreate path, SUCCESS" << endl;
  return true;
}

// Node enhancement step: choose a random deleted edge and generate nodes with a
// gaussian distribution around the edge's midpoint
template <class CFG, class WEIGHT>
void
LazyQuery<CFG, WEIGHT>::NodeEnhance(Roadmap<CFG, WEIGHT>* _rdmp, StatClass& _stats) {
  if(!m_numEnhance || !m_edges.size())
    return;

  if(this->m_debug)
    cout << "*E* In LazyQuery::NodeEnhance. Generated these VIDs:";
  stapl::sequential::vector_property_map<RoadmapGraph<CFG, WEIGHT>, size_t> cmap;
  vector<VID> allVIDs;
  _rdmp->m_pRoadmap->GetVerticesVID(allVIDs);

  for(int i = 0; i < m_numEnhance; i++) {
    size_t index = LRand() % m_edges.size(); // do I need a typecast?
    CFG seed, incr, enhance;
    seed.add(m_edges[index].first, m_edges[index].second);
    seed.divide(seed, 2.0);
    shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(this->m_dmLabel); 
    incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), this->GetMPProblem()->GetEnvironment(), dm);
    enhance.add(seed, incr);
    enhance.SetLabel("Enhance", true);
   
    if(!enhance.InBoundary(this->GetMPProblem()->GetEnvironment(), this->GetMPProblem()->GetEnvironment()->GetBoundary()))
      continue;

    // Add enhance to roadmap and connect
    vector<VID> newVID(1, _rdmp->m_pRoadmap->AddVertex(enhance));
    for(vector<string>::iterator label = this->m_nodeConnectionLabels.begin();
            label != this->m_nodeConnectionLabels.end(); label++) {
      cmap.reset();
      this->GetMPProblem()->GetMPStrategy()->GetConnector()->GetMethod(*label)->Connect(_rdmp, _stats,
          cmap, newVID.begin(), newVID.end(), allVIDs.begin(), allVIDs.end());
    }
    allVIDs.push_back(newVID[0]);
    if(this->m_debug)
      cout << " " << newVID[0];
  }
  if(this->m_debug)
    cout << endl;
}

#endif
