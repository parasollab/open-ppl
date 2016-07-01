
#ifndef TOGGLEPRMSTRATEGY_H_
#define TOGGLEPRMSTRATEGY_H_

#include "MPStrategies/MPStrategyMethod.h"
#include "Utilities/MetricUtils.h"
#include "boost/lambda/lambda.hpp"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Toggle PRM builds two roadmaps, one free and one obstacle
/// @tparam MPTraits Motion planning universe
///
/// Toggle PRM. Constructs a free roadmap and an obstacle roadmap and uses
/// witnesses to aid construction of the two roadmaps.
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class TogglePRMStrategy : public MPStrategyMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;

    TogglePRMStrategy();
    TogglePRMStrategy(MPProblemType* _problem, XMLNode& _node);
    virtual ~TogglePRMStrategy() { }

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

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
    string m_vcLabel;                             // Validity checker
    bool m_priority;                              // Give priority to valid nodes in the queue?
};

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy() : m_priority(false) {
  this->m_name = "TogglePRMStrategy";
}

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy(MPProblemType* _problem, XMLNode& _node) :
MPStrategyMethod<MPTraits>(_problem, _node), m_priority(false) {
  this->m_name = "TogglePRMStrategy";
  ParseXML(_node);
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::ParseXML(XMLNode& _node) {
  m_vcLabel = _node.Read("vcLabel", false, "", "Validity checker method");
  m_priority = _node.Read("priority", false, false, "Whether or not to give priority to valid nodes");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      string generationMethod = child.Read("methodLabel", true, "", "Node connection method");
      int numPerIter = child.Read("number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIter = child.Read("attempts", false, 1, 0, MAX_INT, "Number of attempts");
      m_samplerLabels[generationMethod] = make_pair(numPerIter, attemptsPerIter);
    }
    else if(child.Name() == "Connector")
      m_connectorLabels.push_back(
          child.Read("methodLabel", true, "", "Node connection method"));
    else if(child.Name() == "ColConnector")
      m_colConnectorLabels.push_back(
          child.Read("methodLabel", true, "", "Node connection method"));
    else if(child.Name() == "Evaluator")
      this->m_meLabels.push_back(
          child.Read("methodLabel", true, "", "Evaluation method"));
  }
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Print(ostream& _os) const {
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
  for_each(this->m_meLabels.begin(), this->m_meLabels.end(), _os << _1 << " ");
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
  while(!this->EvaluateMap()) {
    GenerateNodes(queue);

    bool validMap;
    // Loop until map is sufficient, or queue is empty
    while(!(validMap = this->EvaluateMap()) && queue.size()) {
      pair<string, CfgType> p = queue.front();
      queue.pop_front();
      string validity = p.first;
      CfgType cfg = p.second;
      if(this->m_debug) cout << "validity - " << validity << endl;

      if(validity=="valid") { // Valid, add to free roadmap
        VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(cfg);
        allNodesVID.push_back(vid);
        Connect(make_pair("valid", vid), allNodesVID, queue);

        this->CheckNarrowPassageSample(vid);
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

  // Output two final maps
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());
  this->GetBlockRoadmap()->Write(this->GetBaseFilename() + ".block.map", this->GetEnvironment());

  // Output stats
  string str = this->GetBaseFilename() + ".stat";
  ofstream osStat(str.c_str());
  osStat << "Statistics" << endl;
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();

  if(this->m_debug) cout << "\nEnd Finalizing TogglePRMStrategy" << endl;
}

// Helper method, generates nodes
template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::GenerateNodes(deque<pair<string, CfgType> >& _queue) {

  StatClass* stats = this->GetStatClass();
  stringstream clockName;
  clockName << "Node Generation";
  stats->StartClock(clockName.str());

  // Go through list of samplers
  typedef map<string, pair<int,int> >::iterator SIT;
  for(auto&  sampler : m_samplerLabels) {
    typename MPProblemType::SamplerPointer smp =
      this->GetSampler(sampler.first);
    vector<CfgType> outNodes;

    string callee = "TogglePRM::GenerateNodes";
    // Generate nodes for this sampler
    stringstream samplerClockName;
    samplerClockName << "Sampler::" << sampler.first;
    stats->StartClock(samplerClockName.str());

    smp->Sample(sampler.second.first, sampler.second.second, this->m_boundary,
        back_inserter(outNodes), back_inserter(outNodes));

    stats->StopClock(samplerClockName.str());

    if(this->m_debug) {
      cout << "\n\t" << this->GetRoadmap()->GetGraph()->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      stats->PrintClock(samplerClockName.str(), cout);
    }

    // Add nodes to queue
    typedef typename vector<CfgType>::iterator CIT;
    for(auto&  sample : outNodes) {

      // If not validated yet, determine validity
      if(!sample.IsLabel("VALID"))
        this->GetValidityChecker(m_vcLabel)->IsValid(sample, callee);

      // Put nodes into queue, keeping track of validity
      if(sample.GetLabel("VALID")) {
        if(m_priority) // If desired, give valid nodes priority
          _queue.push_front(make_pair("valid", sample));
        else
          _queue.push_back(make_pair("valid", sample));
      }
      else
        _queue.push_back(make_pair("invalid", sample));
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

    // Connect vertex using the connector
    connector->Connect(
        _vid.first=="valid" ? this->GetMPProblem()->GetRoadmap() : this->GetMPProblem()->GetBlockRoadmap(),
        nodesVID.begin(), nodesVID.end(), _allvids.begin(), _allvids.end(),
        true, back_inserter(collision));

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

    stats->StopClock(connectorClockName.str());
    if(this->m_debug) {
      cout << "Freemap: " << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << this->GetRoadmap()->GetGraph()->GetNumCCs() << " connected components" << endl;
      cout << "\t";
      cout << "Blockmap: " << this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << this->GetBlockRoadmap()->GetGraph()->GetNumCCs() << " connected components" << endl;
      cout << "\t";
      stats->PrintClock(connectorClockName.str(), cout);
    }
  }

  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), cout);
}

#endif
