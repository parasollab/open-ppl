// Toggle PRM. Constructs a free roadmap and an obstacle roadmap and uses
// witnesses to aid construction of the two roadmaps.

#ifndef TOGGLEPRMSTRATEGY_H_
#define TOGGLEPRMSTRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "Utilities/MetricUtils.h"
#include "boost/lambda/lambda.hpp"

template<class MPTraits>
class TogglePRMStrategy : public MPStrategyMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;

    TogglePRMStrategy();
    TogglePRMStrategy(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~TogglePRMStrategy() { }

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    // Helper functions for Run()
    void GenerateNodes(deque<pair<string, CfgType> >& _queue);
    void Connect(pair<string, VID> _vid, vector<VID>& _allvids, deque<pair<string, CfgType> >& _queue);

    map<string, pair<int,int> > m_samplerLabels;  // Maps sampler labels to pair<samples, attempts>
    vector<string> m_connectorLabels;             // Connector for free roadmap
    vector<string> m_colConnectorLabels;          // Connector for obstacle roadmap
    vector<string> m_evaluatorLabels;             // Evaluators
    string m_vcLabel;                             // Validity checker
    bool m_priority;                              // Give priority to valid nodes in the queue?
};

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy() : m_priority(false) {
  this->m_name = "TogglePRMStrategy";
}

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy(MPProblemType* _problem, XMLNodeReader& _node) :
MPStrategyMethod<MPTraits>(_problem, _node), m_priority(false) {
  this->m_name = "TogglePRMStrategy";
  ParseXML(_node);
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_vcLabel = _node.stringXMLParameter("vcLabel", false, "", "Validity checker method");
  m_priority = _node.boolXMLParameter("priority", false, false, "Whether or not to give priority to valid nodes");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "Sampler") {
      string generationMethod = citr->stringXMLParameter("methodLabel", true, "", "Node connection method");
      int numPerIter = citr->numberXMLParameter("number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIter = citr->numberXMLParameter("attempts", false, 1, 0, MAX_INT, "Number of attempts");
      m_samplerLabels[generationMethod] = make_pair(numPerIter, attemptsPerIter);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "Connector") {
      string connectMethod = citr->stringXMLParameter("methodLabel", true, "", "Node connection method");
      m_connectorLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "ColConnector") {
      string connectMethod = citr->stringXMLParameter("methodLabel", true, "", "Node connection method");
      m_colConnectorLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    }
    else if(citr->getName() == "Evaluator") {
      string evalMethod = citr->stringXMLParameter("methodLabel", true, "", "Evaluation method");
      m_evaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }
  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::PrintOptions(ostream& _os) const {
  using boost::lambda::_1;
  _os << "\nTogglePRMStrategy::ParseXML:\n";
  _os << "\tSamplers: ";
  for(map<string, pair<int, int> >::const_iterator i = m_samplerLabels.begin(); i != m_samplerLabels.end(); i++)
    _os << i->first << ". Number: " << i->second.first << ", Attempts: " << i->second.second << endl;
  _os << "\tConnectors: ";
  for_each(m_connectorLabels.begin(), m_connectorLabels.end(), _os << _1 << " ");
  _os << "\n\tColConnectors: ";
  for_each(m_colConnectorLabels.begin(), m_colConnectorLabels.end(), _os << _1 << " ");
  _os << "\n\tEvaluators: ";
  for_each(m_evaluatorLabels.begin(), m_evaluatorLabels.end(), _os << _1 << " ");
  _os << "\n\tvcLabel: " << m_vcLabel;
  _os << "\n\tpriority: " << m_priority << endl;
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Initialize() { }

// Runs the Toggle PRM strategy
template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Run() {
  if(this->m_debug) cout << "\nRunning TogglePRMStrategy" << endl;

  // Set up variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  vector<VID> allNodesVID, allColNodesVID;
  deque<pair<string, CfgType> > queue;

  stats->StartClock("Map Generation");

  // Loop until map is sufficient for evaluators
  while(!this->EvaluateMap(m_evaluatorLabels)) {
    GenerateNodes(queue);

    bool validMap;
    // Loop until map is sufficient, or queue is empty
    while(!(validMap = this->EvaluateMap(m_evaluatorLabels)) && queue.size()) {
      pair<string, CfgType> p = queue.front();
      queue.pop_front();
      string validity = p.first;
      CfgType cfg = p.second;
      if(this->m_debug) cout << "validity - " << validity << endl;

      if(validity=="valid") { // Valid, add to free roadmap
        VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(cfg);
        allNodesVID.push_back(vid);
        Connect(make_pair("valid", vid), allNodesVID, queue);
      }
      else { // Invalid, add to obstacle roadmap. Toggle validity while connecting
        VID vid = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->AddVertex(cfg);
        allColNodesVID.push_back(vid);
        this->GetMPProblem()->ToggleValidity();
        Connect(make_pair("invalid", vid), allColNodesVID, queue);
        this->GetMPProblem()->ToggleValidity();
      }
    }

    if(validMap)
      break;
  }

  stats->StopClock("Map Generation");

  if(this->m_debug) {
    stats->PrintClock("MapGeneration", cout);
    cout<<"\nEnd Running TogglePRMStrategy::"<<endl;
  }
}

// Outputs data after Toggle PRM strategy finishes
template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Finalize() {
  if(this->m_debug) cout << "\nFinalizing TogglePRMStrategy::" << endl;

  // Set up variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  // Output two final maps
  string str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  str = this->GetBaseFilename() + ".block.map";
  ofstream osMap2(str.c_str());
  this->GetMPProblem()->GetBlockRoadmap()->Write(osMap2, this->GetMPProblem()->GetEnvironment());
  osMap2.close();

  // Output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream osStat(str.c_str());
  osStat << "Statistics" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();

  if(this->m_debug) cout << "\nEnd Finalizing TogglePRMStrategy" << endl;
}

// Helper method, generates nodes
template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::GenerateNodes(deque<pair<string, CfgType> >& _queue) {

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stringstream clockName;
  clockName << "Node Generation";
  stats->StartClock(clockName.str());

  // Go through list of samplers
  typedef map<string, pair<int,int> >::iterator SIT;
  for(SIT sit = m_samplerLabels.begin(); sit != m_samplerLabels.end(); ++sit) {
    typename MPProblemType::SamplerPointer smp = this->GetMPProblem()->GetSampler(sit->first);
    vector<CfgType> outNodes;

    string callee = "TogglePRM::GenerateNodes";
    // Generate nodes for this sampler
    stringstream samplerClockName;
    samplerClockName << "Sampler::" << sit->first;
    stats->StartClock(samplerClockName.str());

    smp->Sample(this->GetMPProblem()->GetEnvironment(), *stats, sit->second.first, sit->second.second,
        back_inserter(outNodes), back_inserter(outNodes));

    stats->StopClock(samplerClockName.str());

    if(this->m_debug) {
      cout << "\n\t" << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      stats->PrintClock(samplerClockName.str(), cout);
    }

    // Add nodes to queue
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit = outNodes.begin(); cit != outNodes.end(); ++cit) {

      // If not validated yet, determine validity
      if(!(*cit).IsLabel("VALID"))
        this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid(
              *cit, callee);

      // Put nodes into queue, keeping track of validity
      if((*cit).GetLabel("VALID")) {
        if(m_priority) // If desired, give valid nodes priority
          _queue.push_front(make_pair("valid", *cit));
        else
          _queue.push_back(make_pair("valid", *cit));
      }
      else
        _queue.push_back(make_pair("invalid", *cit));
    }
  }
  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), cout);
}

// Helper method to connect a vertex to correct roadmap
template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Connect(pair<string, VID> _vid,
    vector<VID>& _allvids, deque<pair<string, CfgType> >& _queue) {

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stringstream clockName;
  clockName << "Node Connection";
  stats->StartClock(clockName.str());
  stapl::sequential::vector_property_map<typename MPProblemType::GraphType, size_t> cmap;

  // Grab correct set of connectors depending on vertex validity
  vector<string> connectorLabels = (_vid.first=="valid" ? m_connectorLabels : m_colConnectorLabels);

  // Loop through each connector
  for(vector<string>::iterator i = connectorLabels.begin(); i != connectorLabels.end(); ++i) {

    typename MPProblemType::ConnectorPointer connector;
    connector = this->GetMPProblem()->GetConnector(*i);

    stringstream connectorClockName;
    connectorClockName << "Connector::" << *i;
    stats->StartClock(connectorClockName.str());

    vector<VID> nodesVID;
    nodesVID.push_back(_vid.second);
    vector<CfgType> collision, valid;
    cmap.reset();

    // Connect vertex using the connector
    connector->Connect(
        _vid.first=="valid" ? this->GetMPProblem()->GetRoadmap() : this->GetMPProblem()->GetBlockRoadmap(),
        *stats, cmap, nodesVID.begin(), nodesVID.end(), _allvids.begin(), _allvids.end(), back_inserter(collision));

    if(this->m_debug) {
      cout << "\n\nCollision Nodes from connecting: " << collision.size() << endl;
      cout << "Adding " << collision.size() << " collision nodes" << endl;
    }

    // Add witnesses to queue, saving their validities
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit = collision.begin(); cit != collision.end(); ++cit) {
      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")) {
        if(m_priority)
          _queue.push_front(make_pair("valid", *cit));
        else
          _queue.push_back(make_pair("valid", *cit));
      }
      else if(cit->IsLabel("VALID") && !cit->GetLabel("VALID"))
        _queue.push_back(make_pair("invalid", *cit));
      else if(this->m_debug)
        cerr << "In TogglePRMStrategy::Connect(), collision not yet validated?! (Shouldn't happen)" << endl;
    }
    cmap.reset();

    stats->StopClock(connectorClockName.str());
    if(this->m_debug) {
      cout << "Freemap: " << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << get_cc_count(*(this->GetMPProblem()->GetRoadmap()->GetGraph()), cmap) << " connected components" << endl;
      cout << "\t";
      cmap.reset();
      cout << "Blockmap: " << this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << get_cc_count(*(this->GetMPProblem()->GetBlockRoadmap()->GetGraph()), cmap) << " connected components" << endl;
      cout << "\t";
      stats->PrintClock(connectorClockName.str(), cout);
    }
  }

  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), cout);
}

#endif
