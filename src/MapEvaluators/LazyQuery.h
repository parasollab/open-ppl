// First assumes all nodes and edges are valid,
// then checks for validity in the query phase
// and deletes nodes and edges found to be invalid

#ifndef LAZYQUERY_H_
#define LAZYQUERY_H_

#include "Query.h"
#include <functional>
#include <algorithm>

using namespace std;

template<class MPTraits>
class LazyQuery : public Query<MPTraits> {
  
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    
    LazyQuery(const char* _queryFileName = "", string _vcLabel = "") :
      Query<MPTraits>(_queryFileName), m_vcLabel(_vcLabel) { this->SetName("LazyQuery"); }
    LazyQuery(CfgType _start, CfgType _goal, string _vcLabel) :
      Query<MPTraits>(_start, _goal), m_vcLabel(_vcLabel) { this->SetName("LazyQuery"); }
    LazyQuery(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~LazyQuery() {};

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);
   
    // Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
    virtual bool CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    // Does nothing in LazyQuery; used in LazyToggle, for example
    virtual void ProcessInvalidNode(CfgType node) { }

    // Node enhancement step
    virtual void NodeEnhance(RoadmapType* _rdmp);

  protected:
    string m_vcLabel;                // Validity checker
    vector<int> m_resolutions;       // List of resolution multiples to check
    int m_numEnhance;                // How many nodes to generate in node enhancement step
    double m_d;                      // Gaussian distance d
    vector<pair<CfgType, CfgType> > m_edges; // List of removed edges, used in node enhancement
};

template<class MPTraits>
LazyQuery<MPTraits>::LazyQuery(MPProblemType* _problem, XMLNodeReader& _node, bool _warn) :
    Query<MPTraits>(_problem, _node, false) {
  this->SetName("LazyQuery");
  ParseXML(_node);
  if(_warn)
    _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
LazyQuery<MPTraits>::ParseXML(XMLNodeReader& _node) {
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

template<class MPTraits>
void
LazyQuery<MPTraits>::PrintOptions(ostream& _os) {
  Query<MPTraits>::PrintOptions(_os);
  _os << "\tvc label = " << m_vcLabel;
  _os << "\n\tnumEnhance = " << m_numEnhance;
  _os << "\n\td = " << m_d;
  _os << "\n\tresolutions =";
  for(vector<int>::iterator it = m_resolutions.begin(); it != m_resolutions.end(); it++)
    _os << " " << *it;
  _os << endl;
}

// Checks validity of nodes and edges and deletes any invalid ones. Recreates path if valid
template<class MPTraits>
bool
LazyQuery<MPTraits>::CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath) {

  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  Environment* env = this->GetMPProblem()->GetEnvironment();
  StatClass& stats = *(this->GetMPProblem()->GetStatClass());
  string callee = "LazyQuery::CanRecreatePath()";
  CDInfo cdInfo;
  vector<VID> neighbors;
  size_t size = _attemptedPath.size();

  if(this->m_debug) {
    cout << "*L* Starting LAZY with: " << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
      << _rdmp->GetGraph()->get_num_edges()/2 << " edges\n*L* Start vertex checking...\n";
  }

  // Check vertices for validity
  for(size_t i = 0; i < size; i++) {
    // Check from outside to middle
    size_t index = (i%2 ? size - i/2 - 1 : i/2);
    // Skip checks if already checked and valid
    CfgType node = _rdmp->GetGraph()->GetVertex(_attemptedPath[index]);
    if(node.IsLabel("VALID") && node.GetLabel("VALID"))
      continue;
    if(!vcm->IsValid(node, env, stats, cdInfo, &callee)) {
      // Add invalid edges to list
      if(m_numEnhance && !node.IsLabel("Enhance")) {
        typename GraphType::vertex_reference v1 = *(_rdmp->GetGraph()->find_vertex(_attemptedPath[index]));
        for(typename GraphType::adj_edge_iterator aei = v1.begin(); aei != v1.end(); aei++) {
          CfgType target = _rdmp->GetGraph()->GetVertex((*aei).target());
          if(!target.IsLabel("Enhance")) {
            if(this->m_debug)
              cout << "*E* Storing node enhancement edge, VIDs = " << _attemptedPath[index] << ", " << (*aei).target() << endl;
            m_edges.push_back(make_pair(node, target));
          }
        }
      }
      // Delete invalid vertex
      ProcessInvalidNode(node);
      _rdmp->GetGraph()->delete_vertex(_attemptedPath[index]);
      if(this->m_debug)
        cout << "*L* Vertex FAILED: " << _attemptedPath[index] << "\n*L* Ending LAZY with: "
          << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
          << _rdmp->GetGraph()->get_num_edges()/2 << " edges." << endl;
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

      CfgType witness;
      LPOutput<MPTraits> ci;
      typename GraphType::vertex_iterator vi;
      typename GraphType::adj_edge_iterator ei;
      typename GraphType::edge_descriptor ed(_attemptedPath[index], _attemptedPath[index+1]);
      _rdmp->GetGraph()->find_edge(ed, vi, ei);

      // Skip checks if already checked and valid
      if((*ei).property().IsChecked(*resIt))
        continue;

      if(this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel)->IsConnected(
            this->GetMPProblem()->GetEnvironment(), stats, this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel),
            _rdmp->GetGraph()->GetVertex(_attemptedPath[index]),
            _rdmp->GetGraph()->GetVertex(_attemptedPath[index+1]), 
            witness, &ci, this->GetMPProblem()->GetEnvironment()->GetPositionRes() * *resIt,
            this->GetMPProblem()->GetEnvironment()->GetOrientationRes() * *resIt, true, false))
        (*ei).property().SetChecked(*resIt);
      else {
        // Add invalid edge to list
        if(m_numEnhance) {
          CfgType cfg1 = _rdmp->GetGraph()->GetVertex(_attemptedPath[index]);
          CfgType cfg2 = _rdmp->GetGraph()->GetVertex(_attemptedPath[index+1]);
          if(!cfg1.IsLabel("Enhance") && !cfg2.IsLabel("Enhance")) {
            if(this->m_debug)
              cout << "*E* Storing node enhancement edge, VIDs = " << _attemptedPath[index] << ", " <<
                _attemptedPath[index+1] << endl;
            m_edges.push_back(make_pair(cfg1, cfg2));
          }
        }
        // Delete invalid (bidirectional) edge
        ProcessInvalidNode(witness);
        _rdmp->GetGraph()->delete_edge(_attemptedPath[index], _attemptedPath[index+1]);
        _rdmp->GetGraph()->delete_edge(_attemptedPath[index+1], _attemptedPath[index]);
        if(this->m_debug)
          cout << "*L* Edge FAILED: " << _attemptedPath[index] << ", " << _attemptedPath[index+1]
            << "\n*L* Ending LAZY with: " << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
            << _rdmp->GetGraph()->get_num_edges()/2 << " edges." << endl;
        return false;
      }
    }
  }

  // Path all valid, now recreate the path
  _recreatedPath.push_back(_rdmp->GetGraph()->GetVertex(_attemptedPath[0]));
  for(typename vector<VID>::iterator it = _attemptedPath.begin(); (it+1) != _attemptedPath.end(); it++) {
    LPOutput<MPTraits> lpOut;
    this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel)->IsConnected(
        this->GetMPProblem()->GetEnvironment(), stats, this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel),
        _rdmp->GetGraph()->GetVertex(*it),
        _rdmp->GetGraph()->GetVertex(*(it+1)), 
        &lpOut, this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes(), false, true, false);
    _recreatedPath.insert(_recreatedPath.end(), lpOut.path.begin(), lpOut.path.end());
    _recreatedPath.push_back(_rdmp->GetGraph()->GetVertex(*(it+1)));
  }

  if(this->m_debug)
    cout << "*L* Edge checking PASSED\n*L* LAZY: Can recreate path, SUCCESS" << endl;
  return true;
}

// Node enhancement step: choose a random deleted edge and generate nodes with a
// gaussian distribution around the edge's midpoint
template<class MPTraits>
void
LazyQuery<MPTraits>::NodeEnhance(RoadmapType* _rdmp) {
  StatClass& stats = *(this->GetMPProblem()->GetStatClass());
  if(!m_numEnhance || !m_edges.size())
    return;

  if(this->m_debug)
    cout << "*E* In LazyQuery::NodeEnhance. Generated these VIDs:";
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;

  for(int i = 0; i < m_numEnhance; i++) {
    size_t index = LRand() % m_edges.size(); // do I need a typecast?
    CfgType seed, incr, enhance;
    seed = (m_edges[index].first + m_edges[index].second)/2.0;
    DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dmLabel); 
    incr.GetRandomRay(fabs(GaussianDistribution(fabs(m_d), fabs(m_d))), this->GetMPProblem()->GetEnvironment(), dm);
    enhance = seed + incr;
    enhance.SetLabel("Enhance", true);
   
    if(!this->GetMPProblem()->GetEnvironment()->InBounds(enhance))
      continue;

    // Add enhance to roadmap and connect
    VID newVID = _rdmp->GetGraph()->AddVertex(enhance);
    for(vector<string>::iterator label = this->m_nodeConnectionLabels.begin();
            label != this->m_nodeConnectionLabels.end(); label++) {
      cmap.reset();
      this->GetMPProblem()->GetConnector(*label)->Connect(_rdmp, stats, cmap, newVID);
    }
    if(this->m_debug)
      cout << " " << newVID;
  }
  if(this->m_debug)
    cout << endl;
}

#endif
