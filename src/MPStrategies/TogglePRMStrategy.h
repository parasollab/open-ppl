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
    virtual ~TogglePRMStrategy();

    virtual void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);

    virtual void Initialize();
    virtual void Run();
    virtual void Finalize();

  protected:
    //helper functions for operator()
    void GenerateNodes(deque<pair<string, CfgType> >& queue);
    void Connect(pair<string, VID> _vid, vector<VID>& _allvids, deque<pair<string, CfgType> >& queue);

    //data
    map<string, pair<int,int> > m_samplerLabels;
    vector<string> m_connectorLabels;
    vector<string> m_colConnectorLabels;
    vector<string> m_evaluatorLabels;
    string m_vcLabel;
    bool m_priority;
};

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy() : m_priority(false){
  this->m_name = "TogglePRMStrategy";
}

template<class MPTraits>
TogglePRMStrategy<MPTraits>::TogglePRMStrategy(MPProblemType* _problem, XMLNodeReader& _node) :
  MPStrategyMethod<MPTraits>(_problem, _node), m_priority(false){
    this->m_name = "TogglePRMStrategy";
    //read input
    ParseXML(_node);
  }

template<class MPTraits>
TogglePRMStrategy<MPTraits>::~TogglePRMStrategy(){
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr){
    if(citr->getName() == "Sampler") {
      string generationMethod = citr->stringXMLParameter("methodLabel", true, "", "Node Connection Method");
      int numPerIter = citr->numberXMLParameter("number", true, 1, 0, MAX_INT, "Number of samples");
      int attemptsPerIter = citr->numberXMLParameter("attempts", false, 1, 0, MAX_INT, "Number of attempts");
      m_samplerLabels[generationMethod] = make_pair(numPerIter, attemptsPerIter);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "Connector"){
      string connectMethod = citr->stringXMLParameter("methodLabel", true, "", "Node Connection Method");
      m_connectorLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "ColConnector"){
      string connectMethod = citr->stringXMLParameter("methodLabel", true, "", "Node Connection Method");
      m_colConnectorLabels.push_back(connectMethod);
      citr->warnUnrequestedAttributes();
    } 
    else if(citr->getName() == "Evaluator"){
      string evalMethod = citr->stringXMLParameter("methodLabel", true, "", "Evaluation Method");
      m_evaluatorLabels.push_back(evalMethod);
      citr->warnUnrequestedAttributes();
    }
    else
      citr->warnUnknownNode();
  }

  m_vcLabel = _node.stringXMLParameter("vcLabel", false, "", "Label of Validity Checker Method");
  m_priority = _node.boolXMLParameter("priority", false, false, "Priority Queue");
  _node.warnUnrequestedAttributes();

  PrintOptions(cout);
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::PrintOptions(ostream& _os) {
  using boost::lambda::_1;
  _os << "\nTogglePRMStrategy::ParseXML:\n";
  _os << "\tSamplers: "; 
  for(map<string, pair<int, int> >::iterator i = m_samplerLabels.begin();i!=m_samplerLabels.end();i++) 
    _os<<i->first<<" Number:"<<i->second.first<<" Attempts:"<<i->second.second<<endl; 
  _os << "\tConnectors: "; 
  for_each(m_connectorLabels.begin(), m_connectorLabels.end(), _os << _1 << " "); 
  _os << endl;
  _os << "\tColConnectors: "; 
  for_each(m_colConnectorLabels.begin(), m_colConnectorLabels.end(), _os << _1 << " "); 
  _os << endl;
  _os << "\tEvaluators: "; 
  for_each(m_evaluatorLabels.begin(), m_evaluatorLabels.end(), _os << _1 << " "); 
  _os << endl;
  _os << "\tvcLabel: " << m_vcLabel;
  _os << endl;
  _os << "\tpriority: " << m_priority;
  _os << endl;
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Initialize(){
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Run(){
  if(this->m_debug) cout<<"\nRunning TogglePRMStrategy::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  vector<VID> allNodesVID, allCollisionNodesVID, thisIterationNodesVID, thisIterationCollisionNodesVID;

  deque<pair<string, CfgType> > queue;

  stats->StartClock("Map Generation");

  while(!this->EvaluateMap(m_evaluatorLabels)){
    GenerateNodes(queue);

    bool validMap;
    while(!(validMap = this->EvaluateMap(m_evaluatorLabels)) && queue.size()>0){
      pair<string, CfgType> p = queue.front();
      queue.pop_front();
      string validity = p.first;
      CfgType cfg = p.second;
      cout<<"validity - " << validity<<endl;
      if(validity=="valid"){
        VID vid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(cfg);
        allNodesVID.push_back(vid);
        thisIterationNodesVID.push_back(vid);
        Connect(make_pair("valid", vid), allNodesVID, queue);
      }
      else if(validity=="invalid"){
        VID vid = this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->AddVertex(cfg);
        allCollisionNodesVID.push_back(vid);
        thisIterationCollisionNodesVID.push_back(vid);
        this->GetMPProblem()->ToggleValidity();
        Connect(make_pair("invalid", vid), allCollisionNodesVID, queue);
        this->GetMPProblem()->ToggleValidity();
      }
    }

    if(validMap)
      break;
  }

  stats->StopClock("Map Generation");

  if(this->m_debug){
    stats->PrintClock("MapGeneration", cout);
    cout<<"\nEnd Running TogglePRMStrategy::"<<endl;
  }
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Finalize(){
  if(this->m_debug) cout<<"\nFinalizing TogglePRMStrategy::"<<endl;

  //setup variables
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  string str;

  //output final map
  str = this->GetBaseFilename() + ".map";
  ofstream osMap(str.c_str());
  this->GetMPProblem()->GetRoadmap()->Write(osMap, this->GetMPProblem()->GetEnvironment());
  osMap.close();

  str = this->GetBaseFilename() + ".block.map";
  ofstream osMap2(str.c_str());
  this->GetMPProblem()->GetBlockRoadmap()->Write(osMap2, this->GetMPProblem()->GetEnvironment());
  osMap2.close();

  //output stats
  str = this->GetBaseFilename() + ".stat";
  ofstream  osStat(str.c_str());
  osStat << "Statistics" << endl;
  stats->PrintAllStats(osStat, this->GetMPProblem()->GetRoadmap());
  stats->PrintClock("Map Generation", osStat);
  osStat.close();

  if(this->m_debug) cout<<"\nEnd Finalizing TogglePRMStrategy"<<endl;
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::GenerateNodes(deque<pair<string, CfgType> >& queue){
  CDInfo cdInfo;
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stringstream clockName; 
  clockName << "Node Generation"; 
  stats->StartClock(clockName.str());

  typedef map<string, pair<int,int> >::iterator SIT;
  for(SIT sit = m_samplerLabels.begin(); sit != m_samplerLabels.end(); ++sit){
    typename MPProblemType::SamplerPointer smp = this->GetMPProblem()->GetSampler(sit->first);
    vector<CfgType> outNodes, outCollisionNodes;

    string callee = "TogglePRM::GenerateNodes";
    //generate nodes for this node generator method
    stringstream samplerClockName; 
    samplerClockName << "Sampler::" << sit->first;
    stats->StartClock(samplerClockName.str());

    if(this->m_debug) cout << "\n\t";

    smp->Sample(this->GetMPProblem()->GetEnvironment(),*stats,
        sit->second.first, sit->second.second,
        back_inserter(outNodes), back_inserter(outCollisionNodes));

    stats->StopClock(samplerClockName.str());

    if(this->m_debug){
      cout << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_vertices() << " vertices " << endl;
      cout << "\n\t";
      stats->PrintClock(samplerClockName.str(), cout);
    }

    //add nodes to queue
    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit=outNodes.begin(); cit!=outNodes.end(); ++cit){
      if(!(*cit).IsLabel("VALID")){
        !(this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid( 
              *cit, this->GetMPProblem()->GetEnvironment(), *stats, cdInfo, &callee));
      }
      //out nodes mean valid then add them to the real roadmap
      if((*cit).IsLabel("VALID") && ((*cit).GetLabel("VALID"))) {
        if(m_priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }
      //else invalid add to block map
      else{
        queue.push_back(make_pair("invalid", *cit));
      }
    }
    for(CIT cit=outCollisionNodes.begin(); cit!=outCollisionNodes.end(); ++cit){
      if(!(*cit).IsLabel("VALID")){
        !(this->GetMPProblem()->GetValidityChecker(m_vcLabel)->IsValid( 
              *cit, this->GetMPProblem()->GetEnvironment(), *stats, cdInfo, &callee));
      }
      //outCollisionNodes mean INVALID then add to block map
      if((*cit).IsLabel("VALID") && !((*cit).GetLabel("VALID"))) {
        queue.push_back(make_pair("invalid", *cit));
      }
      //else valid add to real map
      else{
        if(m_priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }  
    }
  }
  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), cout);
}

template<class MPTraits>
void
TogglePRMStrategy<MPTraits>::Connect(pair<string, VID> _vid, 
    vector<VID>& _allvids, deque<pair<string, CfgType> >& queue){

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  stringstream clockName; clockName << "Node Connection";
  stats->StartClock(clockName.str());
  stapl::sequential::vector_property_map<typename MPProblemType::GraphType, size_t> cmap;

  vector<string> connectorLabels = _vid.first=="valid" ? m_connectorLabels : m_colConnectorLabels;

  for(vector<string>::iterator i = connectorLabels.begin(); i != connectorLabels.end(); ++i){

    typename MPProblemType::ConnectorPointer connector;
    connector = this->GetMPProblem()->GetConnector(*i);    

    stringstream connectorClockName; 
    connectorClockName << "Connector::" << *i;
    stats->StartClock(connectorClockName.str());

    if(this->m_debug) cout << "\n\t";

    vector<VID> nodesVID;
    nodesVID.push_back(_vid.second);
    vector<CfgType> collision, valid;
    cmap.reset();

    connector->Connect(
        _vid.first=="valid" ? this->GetMPProblem()->GetRoadmap() : this->GetMPProblem()->GetBlockRoadmap(), 
        *stats, cmap,
        nodesVID.begin(), nodesVID.end(), 
        _allvids.begin(), _allvids.end(),
        back_inserter(collision));

    if(this->m_debug){
      cout<<"\nCollision Nodes from connecting::"<<collision.size()<<endl;
      cout<<"Adding "<<collision.size()<<" collision nodes"<<endl;
    }

    typedef typename vector<CfgType>::iterator CIT;
    for(CIT cit=collision.begin(); cit!=collision.end(); ++cit){
      if(cit->IsLabel("VALID") && cit->GetLabel("VALID")){
        if(m_priority)
          queue.push_front(make_pair("valid", *cit));
        else
          queue.push_back(make_pair("valid", *cit));
      }
      else if(cit->IsLabel("VALID") && !cit->GetLabel("VALID")) {
        queue.push_back(make_pair("invalid", *cit));
      }
    }
    cmap.reset();

    stats->StopClock(connectorClockName.str());
    if(this->m_debug){
      cout << "Freemap::" << this->GetMPProblem()->GetRoadmap()->GetGraph()->get_num_edges() << " edges, " 
        << get_cc_count(*(this->GetMPProblem()->GetRoadmap()->GetGraph()), cmap) << " connected components"
        << endl;
      cout << "\t";
      cmap.reset();
      cout << "Blockmap::" << this->GetMPProblem()->GetBlockRoadmap()->GetGraph()->get_num_edges() << " edges, " 
        << get_cc_count(*(this->GetMPProblem()->GetBlockRoadmap()->GetGraph()), cmap) << " connected components"
        << endl;
      cout << "\t";
      stats->PrintClock(connectorClockName.str(), cout);
    }
  }
  stats->StopClock(clockName.str());
  if(this->m_debug) stats->PrintClock(clockName.str(), cout);
}

#endif
