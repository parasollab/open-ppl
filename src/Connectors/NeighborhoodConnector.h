/**Connect nodes in map to their neighbors.
 *Following Algorithm is used:
 *   -# for evry node, cfg1, in roadmap
 *       -# find neighbors N for cfg1
 *       -# lp is a local planner
 *       -# for every node cfg2 in N and numFailures < m_fail
 *           -# test lp.IsConnected(cfg1, cfg2)
 *           -# if connected, add this edge to map, _rm.
 *       -#end for
 *   -# end for,
 */

#ifndef NEIGHBORHOODCONNECTOR_H
#define NEIGHBORHOODCONNECTOR_H

#include "ConnectorMethod.h"

template<class MPTraits>
class NeighborhoodConnector: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::CfgRef CfgRef;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename RoadmapType::GraphType GraphType;

    NeighborhoodConnector(string _nfLabel = "", string _lpLabel = "",
        bool _checkIfSameCC = false, bool _countFailures = false, size_t _fail = 5);
    NeighborhoodConnector(MPProblemType* _problem, XMLNodeReader& _node);

    virtual void PrintOptions(ostream& _os) const;
    virtual void ParseXML(XMLNodeReader& _node);

    template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
          InputIterator1 _itr1First, InputIterator1 _itr1Last,
          InputIterator2 _itr2First, InputIterator2 _itr2Last, OutputIterator _collision) ;

  protected:
    template<typename ColorMap, typename InputIterator, typename OutputIterator>
      void ConnectNeighbors(
          RoadmapType* _rm, StatClass& _stats,
          ColorMap& _cmap, VID _vid,
          InputIterator _closestFirst, InputIterator _closestLast,
          OutputIterator _collision);

  private:
    bool m_checkIfSameCC; //do not test connections inside of a CC if true. Creates a tree.
    bool m_countFailures; //count and limit the failures per iteration
    size_t m_fail; //number of allowed failures per iteration
};

template<class MPTraits>
NeighborhoodConnector<MPTraits>::NeighborhoodConnector(string _nfLabel, string _lpLabel,
    bool _checkIfSameCC, bool _countFailures, size_t _fail) :
  ConnectorMethod<MPTraits>(_nfLabel, _lpLabel),
  m_checkIfSameCC(_checkIfSameCC),
  m_countFailures(_countFailures),
  m_fail(_fail) {
    this->SetName("NeighborhoodConnector");
  }

//Read from XML to get the parameters.
  template<class MPTraits>
NeighborhoodConnector<MPTraits>::NeighborhoodConnector(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem, _node) {
    ParseXML(_node);
  }

template<class MPTraits>
void
NeighborhoodConnector<MPTraits>::ParseXML(XMLNodeReader& _node){
  this->SetName("NeighborhoodConnector");
  m_checkIfSameCC = _node.boolXMLParameter("checkIfSameCC", false, true, "If true, do not connect if edges are in the same CC");
  m_countFailures = _node.boolXMLParameter("countFailures", false, false, "if false, ignore failure count and just attempt k; if true, attempt k neighbors until too many failures detected");
  m_fail = _node.numberXMLParameter("fail", false, 5, 0, 10000, "amount of failed connections allowed before operation terminates");
  _node.warnUnrequestedAttributes();
}

template<class MPTraits>
void
NeighborhoodConnector<MPTraits>::PrintOptions(ostream& _os) const {
  ConnectorMethod<MPTraits>::PrintOptions(_os);
  _os << "\tfail = " << m_fail << endl;
  _os << "\tcountFailures = " << m_countFailures << endl;
}

template<class MPTraits>
template<typename ColorMap, typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
NeighborhoodConnector<MPTraits>::Connect(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last, OutputIterator _collision){

  if(this->m_debug){
    cout << endl;
    PrintOptions(cout);
  }

  NeighborhoodFinderPointer nfptr = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel);

  // the vertices in this iteration are the source for the connection operation
  for(InputIterator1 itr1 = _itr1First; itr1 != _itr1Last; ++itr1){

    // find cfg pointed to by itr1
    VID vid = _rm->GetGraph()->GetVID(itr1);
    CfgRef vCfg = _rm->GetGraph()->GetVertex(itr1);

    if(this->m_debug)
      cout << (itr1 - _itr1First)
        << "\tAttempting connections: VID = "
        << vid << "  --> Cfg = " << vCfg << endl;

    //determine nearest neighbors
    vector<pair<VID, double> > closest;
    nfptr->FindNeighbors(_rm, _itr2First, _itr2Last, vCfg, back_inserter(closest));

    if(this->m_debug){
      cout << "Neighbors | ";
      for(typename vector<pair<VID, double> >::iterator nit = closest.begin(); nit!=closest.end(); ++nit)
        cout << nit->first << " ";
    }

    //test connections through LP
    ConnectNeighbors(_rm, _stats, _cmap, vid, closest.begin(), closest.end(), _collision);
  }
}

template<class MPTraits>
template <typename ColorMap, typename InputIterator, typename OutputIterator>
void
NeighborhoodConnector<MPTraits>::ConnectNeighbors(
    RoadmapType* _rm, StatClass& _stats,
    ColorMap& _cmap, VID _vid,
    InputIterator _closestFirst, InputIterator _closestLast,
    OutputIterator _collision){

  Environment* env = this->GetMPProblem()->GetEnvironment();
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfLabel);
  LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(this->m_lpLabel);
  GraphType* map = _rm->GetGraph();

  LPOutput<MPTraits> lpOutput;
  size_t failure = 0;

  // connect the found k-closest to the current iteration's CfgType
  for(InputIterator itr2 = _closestFirst; itr2 != _closestLast; ++itr2){

    VID v2 = itr2->first;

    if(this->m_debug)
      cout << "\tfailures = " << failure
        << " | VID = " << v2
        << " | dist = " << itr2->second;

    // stopping conditions
    if(this->m_countFailures && failure >= m_fail){
      if(this->m_debug) cout << " | stopping... failures exceeded" << endl;
      break;
    }

    // don't attempt the connection if it already failed once before
    if(this->IsCached(_vid, v2) && !this->GetCached(_vid, v2)){
      if(this->m_debug) {
        cout << " | skipping... this connection already failed once"
          << " | failure incremented" << endl;
      }
      failure++;
      continue;
    }

    // if the edge already exists, so no need to call LP. Count as success.
    if(map->IsEdge(_vid, v2)){
      if(this->m_debug)
        cout << " | edge already exists in roadmap | skipping" << endl;
      continue;
    }

    if(m_checkIfSameCC){
      // if the nodes are in the same connected component count as success
      _cmap.reset();
      if(stapl::sequential::is_same_cc(*map, _cmap, _vid, v2)){
        if(this->m_debug)
          cout << " | nodes in the same connected component | skipping" << endl;
        continue;
      }
    }

    // attempt connection with the local planner
    CfgRef c1 = map->GetVertex(_vid);
    CfgRef c2 = map->GetVertex(v2);

    CfgType col;
    bool connectable = lp->IsConnected(env, _stats, nf->GetDMMethod(),
          c1, c2, col, &lpOutput,
          env->GetPositionRes(), env->GetOrientationRes(), true);
    if(col != CfgType())
      *_collision++ = col;

    //add connection attempt to caches
    this->AddConnectionAttempt(_vid, v2, connectable);

    c1.IncStat("totalConnectionAttempts", 1);
    c2.IncStat("totalConnectionAttempts", 1);

    if(connectable){
      if(this->m_debug) cout << " | connection was successful | success incremented" << endl;
      // increment # of successful connection attempts
      c1.IncStat("succConnectionAttempts", 1);
      c2.IncStat("succConnectionAttempts", 1);
      // if connection was made, add edge and record the successful connection
      _rm->GetGraph()->AddEdge(_vid, v2, lpOutput.edge);
    }
    else {
      if(this->m_debug) cout << " | connection failed | failure incremented" << endl;
      failure++;
    }
  }
}

#endif
