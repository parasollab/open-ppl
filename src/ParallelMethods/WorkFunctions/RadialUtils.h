/////////////////////////
//Class RadiaUtils
////////////////////////

#ifndef RADIALUTILS_H_
#define RADIALUTILS_H_
#include "ParallelMethods/ParallelSBMPHeader.h"

/*
 *  Expand for Blind RRT where three different values can be returned: OUTOFBOUNDARY, INVALID, VALID
 * */
namespace ExpansionType {
  enum Expansion {
    IN_COLLISION,
    NO_COLLISION,
    NO_EXPANSION,
  };

  string GetExpansionTypeString(Expansion expansion);
}

/*
 * Blind RRT Expand
 * Expands up to delta distance or when reaching out of boundary. If a collision is
 * encountered, it records the last valid sample and keeps growing.
 */
template<class MPTraits>
ExpansionType::Expansion
BlindRRTExpand(typename MPTraits::MPProblemType* _mp,
    string _vc, string _dm,
    typename MPTraits::CfgType _start,
    typename MPTraits::CfgType _dir,
    vector< pair<typename MPTraits::CfgType, int> >& _newCfgs, // pair = Cfg , weight. weight is the distance from the Cfg to the start Cfg
    double _delta,
    double _posRes, double _oriRes){

  //Setup...primarily for collision checks that occur later on
  string callee("RRTUtility::BlindRRTExpand");
  Environment* env = _mp->GetEnvironment();
  typename MPTraits::MPProblemType::DistanceMetricPointer dm = _mp->GetDistanceMetric(_dm);
  typename MPTraits::MPProblemType::ValidityCheckerPointer vc = _mp->GetValidityChecker(_vc);

  typedef typename MPTraits::CfgType CfgType;
  typename MPTraits::CfgType incr, tick = _start, previous = _start;
  // if any collision is encountered, collision becomes true
  bool collision=false, outOfBoundary=false;
  // this tracks collision changes along the expansion
  bool prevCollision = !vc->IsValid(_start, callee);
  int nTicks, ticker = 0;
  pair<CfgType, int> cfgWeight;
  incr.FindIncrement(tick,_dir,&nTicks, _posRes, _oriRes);

  //Move out from start towards dir, bounded by number of ticks allowed at a given resolution.  Delta + obsDist are
  //given to the function, and are user defined.
  while(!outOfBoundary && dm->Distance(_start,tick) <= _delta && ticker <= nTicks) {
    previous = tick;
    tick += incr; //Increment tick
    ++ticker;

    if(!env->InBounds(tick))
      outOfBoundary = true; // Expansion is out of boundary, return previous tick
    else if (!(vc->IsValid(tick, callee))) {
      collision = true; //Found a collision, activate flag
      if(!prevCollision) {
        // record the previous sample since it is valid
        cfgWeight = make_pair(previous, ticker - 1); // previous is one tick behind
        _newCfgs.push_back(cfgWeight);

        // TODO do we really want to add in collision nodes???
        /*
        cfgWeight = make_pair(tick, ticker);
        _newCfgs.push_back(cfgWeight);
        */
        prevCollision = true;
      }
    } else {

      if(prevCollision) {

        /*
        // TODO do we really want to add in collision nodes???
        cfgWeight = make_pair(previous, ticker - 1); // previous is one tick behind
        _newCfgs.push_back(cfgWeight);
        */

        // record this sample since the previous was in collision
        cfgWeight = make_pair(tick, ticker);
        _newCfgs.push_back(cfgWeight);

        prevCollision = false;
      }
    }
  }
  if(previous != _start){ //Did we go anywhere?

    cfgWeight = make_pair(previous, ticker);
    // TODO Cesar: previous might get added in the previous step
    if(previous != _newCfgs.back().first)
      _newCfgs.push_back(cfgWeight);//Last Cfg pushed back is the final tick allowed

    if (collision) {    // encountered collision in the way
      return ExpansionType::IN_COLLISION;

    } else {            // No collision and not out of boundary
      return ExpansionType::NO_COLLISION;
    }

  }
  // Didn't find a place to go :(
  else
    return ExpansionType::NO_EXPANSION;
}

template<class MPTraits>
class RadialUtils {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef typename MPProblemType::RoadmapType RoadmapType;
  typedef typename MPProblemType::GraphType GraphType;
  typedef typename GraphType::GRAPH LocalGraphType;
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
  typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
  typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
  typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
  typedef pair<size_t, VID> CCType;
  typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalTreeType;

  private:
  MPProblemType* m_problem;
  size_t m_numCCIters; // iterations of the CC connection phase
  string m_dm;
  string m_vc;
  string m_nf;
  string m_CCconnection; // CC connection policy
  double m_delta;
  double m_minDist;
  LocalTreeType* m_localTree; // the tree we are working on (parallel)
  string m_callee = "RadialUtils";
  bool m_debug;

  public:

  RadialUtils() {}

  RadialUtils(MPProblemType* _problem, LocalTreeType* _localTree, string _dm, string _vc, string _nf, string _CCconnection,
      double _delta, double _minDist, size_t _numCCIters=0, bool _debug=false) {

    m_problem = _problem;
    m_numCCIters = _numCCIters;
    m_dm = _dm;
    m_delta = _delta;
    m_vc = _vc;
    m_nf = _nf;
    m_CCconnection = _CCconnection;
    m_minDist = _minDist;
    m_debug = _debug;

/*
    if (_localTree == NULL)
      m_localTree = m_problem->GetRoadmap()->GetGraph();

    else  */
      m_localTree = _localTree;
  }

  // A wrapper for checking validity since sometimes the label is not set
  bool IsValid(ValidityCheckerPointer _vc, CfgType& _cfg) {
    if (!_cfg.IsLabel("VALID"))
      _vc->IsValid(_cfg, m_callee);
    return _cfg.GetLabel("VALID");
  }

  ////////////////////////////
  //  AddVertex
  ////////////////////////////
  inline VID AddVertex(CfgType _cfg) const {
    VID newVID = m_problem->GetRoadmap()->GetGraph()->add_vertex(_cfg);
    m_localTree->add_vertex(newVID, _cfg);
    if(m_debug)  VDAddNode(_cfg);
    return newVID;
  }

  ////////////////////////////
  //  ADD EDGE
  ////////////////////////////
  // this is used with no debugging purposes on parallel so it does not write out to the vizmo
  // or graph
  void AddEdge(VID _vid1, VID _vid2, int _weight, vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights) {

    if((_vid1 == 0 || _vid2 == 0)
#ifdef _PARALLEL
        && stapl::get_location_id() != 0
#endif
        ) {
      _pendingEdges.push_back(make_pair(_vid1,_vid2));
      _pendingWeights.push_back(_weight);
      return;
    }

    WeightType weight("RRTExpand", _weight);
    // m_problem->GetRoadmap()->GetGraph()->AddEdge(_vid1, _vid2, weights);
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
#ifdef _PARALLEL
    globalTree->add_edge_async(_vid1, _vid2, weight);
    globalTree->add_edge_async(_vid2, _vid1, weight);
#else
    globalTree->AddEdge(_vid1, _vid2, weight);
#endif
    m_localTree->add_edge(_vid1, _vid2);
    m_localTree->add_edge(_vid2, _vid1);
  }

  ////////////////////////////
  //  AddEdgeDebug
  ////////////////////////////
  // Used when we want to see the results on vizmo
  void AddEdgeDebug(VID _vid1, VID _vid2, CfgType _cfg1, CfgType _cfg2, int _weight, vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights) {
    AddEdge(_vid1, _vid2, _weight, _pendingEdges, _pendingWeights);
    VDAddEdge(_cfg1, _cfg2);
  }


  ////////////////////////////
  //  ExpandTree Wrapper for Sequential Blind RRT
  ////////////////////////////
  int ExpandTree(vector<VID>& _currBranch, CfgType& _dir)  {
    NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);
    RoadmapType* rdmp = m_problem->GetRoadmap();
    StatClass* stats = m_problem->GetStatClass();

    vector<pair <VID, VID> > dummy1;
    vector<int> dummy2;

    vector<pair<VID,double>> kClosest;
    vector<CfgType> cfgs;

    string kcloseClockName = "kclosest time ";
    stats->StartClock(kcloseClockName);

    nf->FindNeighbors(rdmp, _dir, back_inserter(kClosest));

    stats->StopClock(kcloseClockName);

    VID nearestVID = kClosest[0].first;
    CfgType nearest;

    nearest = rdmp->GetGraph()->GetVertex(nearestVID);

    return ExpandTree(_currBranch, nearestVID, nearest, _dir,dummy1, dummy2);
  }

  ////////////////////////////
  //  ExpandTree
  ////////////////////////////
  int ExpandTree(vector<VID>& _currBranch, VID _nearestVID, CfgType& _nearest, CfgType& _dir,
      vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights)  {
    Environment* env = m_problem->GetEnvironment();
    DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    VID newVID = INVALID_VID;
    string callee("RadialRRT::ExpandTree");

    ExpansionType::Expansion expansion;
    vector<pair<CfgType, int> > expansionCfgs;  // this will contain all cfgs from start to goal inclusive
    expansionCfgs.push_back(make_pair(_nearest, 0));
    expansion = BlindRRTExpand<MPTraits>(m_problem, m_vc, m_dm,
        _nearest, _dir, expansionCfgs, m_delta,
        env->GetPositionRes(), env->GetOrientationRes());


    if (expansion == ExpansionType::NO_EXPANSION) {
      return 0;
    }

    CfgType& newCfg = expansionCfgs.back().first; // last cfg in the returned array is delta away from nearest
    int nodesAdded = 0;
    // If good to go, add to roadmap
    if(dm->Distance(newCfg, _nearest) >= m_minDist ) {

      // Adding Nodes
      vector<VID> expansionVIDs;
      expansionVIDs.push_back(_nearestVID);

      // we already added startCfg remember?
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {
        VID newVID = INVALID_VID;
        // Expansion returns both valid and invalid  so we can track valid edges,
        // but only add valid nodes to the tree
        if (IsValid(vc, expansionCfgs[i].first)) {
          newVID = AddVertex(expansionCfgs[i].first);
          _currBranch.push_back(newVID);/// branch is being replaced as local tree, remove this
        }

        expansionVIDs.push_back(newVID );

      }

      pair<WeightType, WeightType> weights;
      // Adding Edges
      int weight;
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {

        LPOutput<MPTraits> lpout;
        CfgType col;
        if(expansionVIDs[i-1] != INVALID_VID && expansionVIDs[i] != INVALID_VID &&
            IsValid(vc, expansionCfgs[i-1].first) &&
            IsValid(vc, expansionCfgs[i].first) &&
            m_problem->GetLocalPlanner("")->IsConnected(
              expansionCfgs[i-1].first,
              expansionCfgs[i].first, col, &lpout,
              env->GetPositionRes(), env->GetOrientationRes(), true)) {
          weight = expansionCfgs[i].second - expansionCfgs[i-1].second; // Edge weight

          if(!m_debug) AddEdge(expansionVIDs[i-1], expansionVIDs[i], weight, _pendingEdges, _pendingWeights);
          else AddEdgeDebug(expansionVIDs[i-1], expansionVIDs[i], expansionCfgs[i-1].first, expansionCfgs[i].first, weight, _pendingEdges, _pendingWeights);

        }
      }

      nodesAdded=  expansionCfgs.size() - 1;    // substract one cause the start already belonged to the tree
    }

    return nodesAdded;

  }


  ////////////////////////////
  //  ConnectCCs
  ////////////////////////////
  // TODO: Debug timer if there is need for using it!
  void ConnectCCs() {
    if (m_localTree == NULL) {
      cout << "Error! Did not Set Local Tree using native view" << endl;
      exit(-1);
    }
    //Setup MP variables
    StatClass* stats = m_problem->GetStatClass();
    RoadmapType* rdmp = m_problem->GetRoadmap();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
    ConnectorPointer pConnection = m_problem->GetConnector("RRTConnect");
    CDInfo cdInfo;
    string callee("BlindRRT::RemoveInvalidNodes");

    stringstream clockName; clockName << "Component Connection";
    stats->StartClock(clockName.str());

    ColorMap colorMap;
    vector< CCType > ccs;
    colorMap.reset();
    stapl::sequential::get_cc_stats(*m_localTree, colorMap, ccs);

    if(ccs.size()==1) return;
    if(m_debug) cout << "Number of CCs before CC Connection: " << ccs.size() << endl;
    vector<VID> cc1;
    vector<VID> cc2;
    VID cc1VID;
    VID cc2VID;

    //size_t connectSwitch = 0;
    size_t iters = 0;
    size_t failures = 0;
    size_t maxFailures = 100;

    // instead of a certain number of iterations, make it proportional to the number of CCs we have
    m_numCCIters = ccs.size() * m_numCCIters;

    while(ccs.size() > 1 && iters <= m_numCCIters && failures < maxFailures) {
      // get a random CC to work on
      int rand1 = LRand() % ccs.size();
      cc1VID = ccs[ rand1 ].second;

      colorMap.reset();
      stapl::sequential::get_cc(*m_localTree,colorMap,cc1VID,cc1);


      if (cc1.size() == 1) {
        CfgType cfg  =   (*(m_localTree->find_vertex(cc1[0]))).property();

        if (!IsValid(vc, cfg)) {
          failures++;
          continue;

        }
      }
      // do we need random selection?
      if (m_CCconnection == "Random") {

        int rand2 = LRand() % ccs.size();
        if (rand1 == rand2) continue;
        cc2VID = ccs[ rand2 ].second;

      // do we need mixed selection?
      }
      /*else if(m_CCconnection == "Mixed") {
        if (connectSwitch % 10 == 0) {
          cc2VID = GetClosestNodeToCentroid(cc1VID, ccs, colorMap);
        }
        else if (connectSwitch % 5 == 0) {
          VID randomNode = cc1[LRand() % cc1.size()];
          cc2VID = GetClosestNodeToNode(randomNode, cc1VID);
        }
        else if (connectSwitch % 3 == 0) {
          int rand2 = LRand() % ccs.size();
          if (rand1 == rand2) continue;
          cc2VID = ccs[ rand2 ].second;
        }
        else {
          CfgType centroid = GetCentroid(cc1);
          cc2VID = GetClosestCentroidToCentroid(centroid, cc1VID, ccs, colorMap);
        }
        connectSwitch++;
      }*/
      else {
        cc2VID = GetClosest(cc1VID, ccs, m_CCconnection);
      }

      if (cc2VID == INVALID_VID )
        continue;

      colorMap.reset();
      stapl::sequential::get_cc(*m_localTree,colorMap,cc2VID,cc2);

      // Maybe this is an invalid node, don't use it
      if (cc2.size() == 1) {

        //CfgType cfg  =   (*(globalTre->distribution().container_manager().begin()->find_vertex(cc2[0]))).property();
        //CfgType cfg  =   (*(globalTre->find_vertex(cc2[0]))).property();
        //CfgType cfg  =   globalTre->GetCfg(cc2[0]);
        CfgType cfg  =   (*(m_localTree->find_vertex(cc2[0]))).property();

        if (!IsValid(vc, cfg)) {
          failures++;
          continue;

        }

      }
      // We got a pair of CCs, attempt to Connect them!
#ifdef _PARALLEL
      pConnection->SetLocalGraph(m_localTree);
#endif
      pConnection->Connect(rdmp, *stats, colorMap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end()) ;

      iters++;
      colorMap.reset();
      stapl::sequential::get_cc_stats(*m_localTree, colorMap, ccs);

    }

    if(m_debug) cout << "Number of CCs after CC Connection: " << ccs.size() << endl;
    stats->StopClock(clockName.str());

  }


  ////////////////////////////
  //  RemoveInvalidNodes
  ////////////////////////////
  // Rewrite function to take iterate through local tree instead of allVIDs vector
  void RemoveInvalidNodes(vector<VID>& _allVIDs) {

    RoadmapType* rdmp = m_problem->GetRoadmap();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    CDInfo  cdInfo;
    string callee("BlindRRT::RemoveInvalidNodes");

    for (size_t i=0; i<_allVIDs.size(); i++) {
      VID vid = _allVIDs[i];
      //CfgType cfg = (*(globalTre->find_vertex(vid))).property();
      //CfgType cfg = globalTre->GetCfg(vid);
      //CfgType cfg = (*(globalTre->distribution().container_manager().begin()->find_vertex(vid))).property();
      CfgType cfg = (*(m_localTree->find_vertex(vid))).property();

      if (!IsValid(vc, cfg)) {
        rdmp->GetGraph()->delete_vertex(_allVIDs[i]);
        m_localTree->delete_vertex(_allVIDs[i]);
        //VDRemoveNode(cfg);
      }
    }
  }


  ////////////////////////////
  //  NodeToNode
  ////////////////////////////
  // Absolute closest pair of nodes
  ////////////////////////////
  double
    GetDistanceNodeToNode(vector<VID>& _sourceCC, vector<VID>& _targetCC) const {
      NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);
      RoadmapType* rdmp = m_problem->GetRoadmap();
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      vector<pair<pair<VID,VID>, double>> closestPair;
      nf->FindNeighborPairs(rdmp, _sourceCC.begin(), _sourceCC.end(), _targetCC.begin(), _targetCC.end(), back_inserter(closestPair));
      if (closestPair.empty())
        return INVALID_VID;

      VID vid1 = closestPair[0].first.first;
      VID vid2 = closestPair[0].first.second;

      CfgType cfg1 = (*(m_localTree->find_vertex(vid1))).property();
      CfgType cfg2 = (*(m_localTree->find_vertex(vid2))).property();

      return dm->Distance(cfg1, cfg2);
    }

  ////////////////////////////
  //  NodeToCentroid
  ////////////////////////////
  //  Calculate the centroid of target CC, find the closest node to sourceCC,
  ////////////////////////////
  double
    GetDistanceNodeToCentroid(vector<VID>& _sourceCC, vector<VID>& _targetCC) const {

      RoadmapType* rdmp = m_problem->GetRoadmap();
      NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      CfgType targetCentroid = GetCentroid(_targetCC);
      vector<pair<VID, double>> closestNode;
      nf->FindNeighbors(rdmp, _sourceCC.begin(), _sourceCC.end(), targetCentroid, back_inserter(closestNode));
      if (closestNode.empty())
        return INVALID_VID;

      VID closest = closestNode[0].first;
      CfgType cfg  =   (*(m_localTree->find_vertex(closest))).property();
      return dm->Distance(cfg, targetCentroid);

    }

  ////////////////////////////
  //  CentroidToNode
  ////////////////////////////
  //  Get the closest node to centroid of sourceCC
  ////////////////////////////
  double
    GetDistanceCentroidToNode(vector<VID>& _sourceCC, vector<VID>& _targetCC) const {

      RoadmapType* rdmp = m_problem->GetRoadmap();
      NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      CfgType sourceCentroid = GetCentroid(_sourceCC);
      vector<pair<VID, double>> closestNode;
      nf->FindNeighbors(rdmp, _targetCC.begin(), _targetCC.end(), sourceCentroid, back_inserter(closestNode));
      if (closestNode.empty())
        return INVALID_VID;

      VID closest = closestNode[0].first;
      CfgType cfg  = (*(m_localTree->find_vertex(closest))).property();
      return dm->Distance(sourceCentroid, cfg);
    }

  ////////////////////////////
  //  CentroidToCentroid
  ////////////////////////////
  //  Distance between centroids
  ////////////////////////////
  double
    GetDistanceCentroidToCentroid(vector<VID>& _sourceCC, vector<VID>& _targetCC) const {
      CfgType sourceCentroid = GetCentroid(_sourceCC);
      CfgType targetCentroid = GetCentroid(_targetCC);
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      return dm->Distance(sourceCentroid, targetCentroid);
    }

  VID
    GetClosest(VID _ccVID, vector<CCType> _ccs, string _criteria) const {
      if (m_localTree == NULL) {
        cout << "Error! Did not Set Local Tree using native view" << endl;
        exit(-1);
      }

      // here we are measuring the distances using the policy/criteria supplied
      // we are choosing the CC with the minimum distance
      ColorMap colorMap;
      vector<VID> sourceCC;
      vector<VID> targetCC;
      double currMinDist = MAX_DBL;
      VID currMinVID = INVALID_VID;
      double dist;
      colorMap.reset();
      stapl::sequential::get_cc(*m_localTree,colorMap,_ccVID,sourceCC);

      for (int i=0; i<_ccs.size(); i++) {
        if(_ccs[i].second == _ccVID)
          continue;

        colorMap.reset();
        stapl::sequential::get_cc(*m_localTree,colorMap,_ccs[i].second,targetCC);

        /////////////////////////////////
        // Call the appropriate method
        /////////////////////////////////
        if(_criteria == "NodeToNode")
          dist = GetDistanceNodeToNode(sourceCC, targetCC);

        else if (_criteria == "NodeToCentroid")
          dist = GetDistanceNodeToCentroid(sourceCC, targetCC);

        else if (_criteria == "CentroidToNode")
          dist = GetDistanceCentroidToNode(sourceCC, targetCC);

        else if (_criteria == "CentroidToCentroid")
          dist = GetDistanceNodeToNode(sourceCC, targetCC);

        else {
          cout << "Unknown CC connection type: " << _criteria << endl;
          exit(-1);
        }

        if (dist < currMinDist) {
          currMinDist = dist;
          currMinVID = _ccs[i].second;
        }

        targetCC.clear();
      }
      return currMinVID;
    }


  CfgType
    GetCentroid(vector<VID>& _cc) const {
      CfgType center;
      for(size_t i = 0; i < _cc.size(); i++) {
        CfgType cfg = (*(m_localTree->find_vertex(_cc[i]))).property();
        center += cfg;
      }
      center /= _cc.size();
      return center;
    }


};


#endif

