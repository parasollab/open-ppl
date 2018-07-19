#ifndef TOGGLE_PRM_STRATEGY_H_
#define TOGGLE_PRM_STRATEGY_H_

#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "Utilities/MetricUtils.h"
#include "boost/lambda/lambda.hpp"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Toggle PRM builds two roadmaps, one free and one obstacle
///
/// Toggle PRM. Constructs a free roadmap and an obstacle roadmap and uses
/// witnesses to aid construction of the two roadmaps.
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TogglePRMStrategy : public MPStrategyMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType     CfgType;
    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    TogglePRMStrategy();
    TogglePRMStrategy(XMLNode& _node);
    virtual ~TogglePRMStrategy() { }

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(std::ostream& _os) const;

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    // Helper functions for Run()
    void GenerateNodes(deque<std::pair<std::string, CfgType> >& _queue);
    void Connect(std::pair<std::string, VID> _vid, std::vector<VID>& _allvids, deque<std::pair<std::string, CfgType> >& _queue);

    std::map<std::string, std::pair<int,int> > m_samplerLabels;  // Maps sampler labels to pair<samples, attempts>
    std::vector<std::string> m_connectorLabels;             // Connector for free roadmap
    std::vector<std::string> m_colConnectorLabels;          // Connector for obstacle roadmap
    std::string m_vcLabel;                             // Validity checker
    bool m_priority;                              // Give priority to valid nodes in the queue?
};

template <typename MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy() : m_priority(false) {
  this->SetName("TogglePRMStrategy");
}

template <typename MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy(XMLNode& _node) :
MPStrategyMethod<MPTraits>(_node), m_priority(false) {
  this->SetName("TogglePRMStrategy");
  ParseXML(_node);
}

template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::ParseXML(XMLNode& _node) {
  m_vcLabel = _node.Read("vcLabel", false, "", "Validity checker method");
  m_priority = _node.Read("priority", false, false, "Whether or not to give priority to valid nodes");

  for(auto& child : _node) {
    if(child.Name() == "Sampler") {
      std::string generationMethod = child.Read("methodLabel", true, "", "Node connection method");
      int numPerIter = child.Read("number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIter = child.Read("attempts", false, 1, 0, MAX_INT, "Number of attempts");
      m_samplerLabels[generationMethod] = std::make_pair(numPerIter, attemptsPerIter);
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

template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::Print(std::ostream& _os) const {
  using boost::lambda::_1;
  _os << "\nTogglePRMStrategy::ParseXML:\n";
  _os << "\tSamplers: ";
  for(std::map<std::string, std::pair<int, int> >::const_iterator i = m_samplerLabels.begin(); i != m_samplerLabels.end(); i++)
    _os << i->first << ". Number: " << i->second.first << ", Attempts: " << i->second.second << std::endl;
  _os << "\tConnectors: ";
  for_each(m_connectorLabels.begin(), m_connectorLabels.end(), _os << _1 << " ");
  _os << "\n\tColConnectors: ";
  for_each(m_colConnectorLabels.begin(), m_colConnectorLabels.end(), _os << _1 << " ");
  _os << "\n\tEvaluators: ";
  for_each(this->m_meLabels.begin(), this->m_meLabels.end(), _os << _1 << " ");
  _os << "\n\tvcLabel: " << m_vcLabel;
  _os << "\n\tpriority: " << m_priority << std::endl;
}

template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::Initialize() { }

// Runs the Toggle PRM strategy
template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::Run() {
  if(this->m_debug) std::cout << "\nRunning TogglePRMStrategy" << std::endl;

  // Set up variables
  StatClass* stats = this->GetStatClass();
  std::vector<VID> allNodesVID, allColNodesVID;
  deque<std::pair<std::string, CfgType> > queue;

  stats->StartClock("Map Generation");

  // Loop until map is sufficient for evaluators
  while(!this->EvaluateMap()) {
    GenerateNodes(queue);

    bool validMap;
    // Loop until map is sufficient, or queue is empty
    while(!(validMap = this->EvaluateMap()) && queue.size()) {
      std::pair<std::string, CfgType> p = queue.front();
      queue.pop_front();
      std::string validity = p.first;
      CfgType cfg = p.second;
      if(this->m_debug) std::cout << "validity - " << validity << std::endl;

      if(validity=="valid") { // Valid, add to free roadmap
        VID vid = this->GetRoadmap()->GetGraph()->AddVertex(cfg);
        allNodesVID.push_back(vid);
        Connect(std::make_pair("valid", vid), allNodesVID, queue);

        this->CheckNarrowPassageSample(vid);
      }
      else { // Invalid, add to obstacle roadmap. Toggle validity while connecting
        VID vid = this->GetBlockRoadmap()->GetGraph()->AddVertex(cfg);
        allColNodesVID.push_back(vid);
        this->GetMPLibrary()->ToggleValidity();
        Connect(std::make_pair("invalid", vid), allColNodesVID, queue);
        this->GetMPLibrary()->ToggleValidity();
      }
    }

    if(validMap)
      break;
  }

  stats->StopClock("Map Generation");

  if(this->m_debug) {
    stats->PrintClock("MapGeneration", std::cout);
    std::cout<<"\nEnd Running TogglePRMStrategy::"<<std::endl;
  }
}

// Outputs data after Toggle PRM strategy finishes
template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::Finalize() {
  if(this->m_debug) std::cout << "\nFinalizing TogglePRMStrategy::" << std::endl;

  // Output two final maps
  this->GetRoadmap()->Write(this->GetBaseFilename() + ".map", this->GetEnvironment());
  this->GetBlockRoadmap()->Write(this->GetBaseFilename() + ".block.map", this->GetEnvironment());

  // Output stats
  std::string str = this->GetBaseFilename() + ".stat";
  std::ofstream osStat(str.c_str());
  osStat << "Statistics" << std::endl;
  StatClass* stats = this->GetStatClass();
  stats->PrintAllStats(osStat, this->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();

  if(this->m_debug) std::cout << "\nEnd Finalizing TogglePRMStrategy" << std::endl;
}

// Helper method, generates nodes
template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::GenerateNodes(deque<std::pair<std::string, CfgType> >& _queue) {

  StatClass* stats = this->GetStatClass();
  std::stringstream clockName;
  clockName << "Node Generation";
  stats->StartClock(clockName.str());

  // Go through list of samplers
  typedef std::map<std::string, std::pair<int,int> >::iterator SIT;
  for(auto&  sampler : m_samplerLabels) {
    auto smp = this->GetSampler(sampler.first);
    std::vector<CfgType> outNodes;

    std::string callee = "TogglePRM::GenerateNodes";
    // Generate nodes for this sampler
    std::stringstream samplerClockName;
    samplerClockName << "Sampler::" << sampler.first;
    stats->StartClock(samplerClockName.str());

    smp->Sample(sampler.second.first, sampler.second.second,
        this->GetEnvironment()->GetBoundary(),
        std::back_inserter(outNodes), std::back_inserter(outNodes));

    stats->StopClock(samplerClockName.str());

    if(this->m_debug) {
      std::cout << "\n\t" << this->GetRoadmap()->GetGraph()->get_num_vertices() << " vertices " << std::endl;
      std::cout << "\n\t";
      stats->PrintClock(samplerClockName.str(), std::cout);
    }

    // Add nodes to queue
    for(auto&  sample : outNodes) {

      // If not validated yet, determine validity
      if(!sample.IsLabel("VALID"))
        this->GetValidityChecker(m_vcLabel)->IsValid(sample, callee);

      // Put nodes into queue, keeping track of validity
      if(sample.GetLabel("VALID")) {
        if(m_priority) // If desired, give valid nodes priority
          _queue.push_front(std::make_pair("valid", sample));
        else
          _queue.push_back(std::make_pair("valid", sample));
      }
      else
        _queue.push_back(std::make_pair("invalid", sample));
    }
  }
  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), std::cout);
}

// Helper method to connect a vertex to correct roadmap
template <typename MPTraits>
void
TogglePRMStrategy<MPTraits>::Connect(std::pair<std::string, VID> _vid,
    std::vector<VID>& _allvids, deque<std::pair<std::string, CfgType> >& _queue) {

  StatClass* stats = this->GetStatClass();
  std::stringstream clockName;
  clockName << "Node Connection";
  stats->StartClock(clockName.str());

  // Grab correct set of connectors depending on vertex validity
  std::vector<std::string> connectorLabels = (_vid.first=="valid" ? m_connectorLabels : m_colConnectorLabels);

  // Loop through each connector
  for(std::vector<std::string>::iterator i = connectorLabels.begin(); i != connectorLabels.end(); ++i) {

    auto connector = this->GetConnector(*i);

    std::stringstream connectorClockName;
    connectorClockName << "Connector::" << *i;
    stats->StartClock(connectorClockName.str());

    std::vector<VID> nodesVID;
    nodesVID.push_back(_vid.second);
    std::vector<CfgType> collision, valid;

    // Connect vertex using the connector
    connector->Connect(
        _vid.first=="valid" ? this->GetRoadmap() : this->GetBlockRoadmap(),
        nodesVID.begin(), nodesVID.end(), _allvids.begin(), _allvids.end(),
        true, std::back_inserter(collision));

    if(this->m_debug) {
      std::cout << "\n\nCollision Nodes from connecting: " << collision.size() << std::endl;
      std::cout << "Adding " << collision.size() << " collision nodes" << std::endl;
    }

    // Add witnesses to queue, saving their validities
    for(auto cit = collision.begin(); cit != collision.end(); ++cit) {
      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")) {
        if(m_priority)
          _queue.push_front(std::make_pair("valid", *cit));
        else
          _queue.push_back(std::make_pair("valid", *cit));
      }
      else if(cit->IsLabel("VALID") && !cit->GetLabel("VALID"))
        _queue.push_back(std::make_pair("invalid", *cit));
      else if(this->m_debug)
        std::cerr << "In TogglePRMStrategy::Connect(), collision not yet validated?! (Shouldn't happen)" << std::endl;
    }

    stats->StopClock(connectorClockName.str());
    if(this->m_debug) {
      std::cout << "Freemap: " << this->GetRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << this->GetRoadmap()->GetGraph()->GetNumCCs() << " connected components" << std::endl;
      std::cout << "\t";
      std::cout << "Blockmap: " << this->GetBlockRoadmap()->GetGraph()->get_num_edges() << " edges, "
        << this->GetBlockRoadmap()->GetGraph()->GetNumCCs() << " connected components" << std::endl;
      std::cout << "\t";
      stats->PrintClock(connectorClockName.str(), std::cout);
    }
  }

  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), std::cout);
}

#endif
