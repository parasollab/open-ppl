
#include "ParallelMethods/ParallelSBMPHeader.h"

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
  size_t m_numCCIters;
  string m_dm;
  string m_vc;
  string m_nf;
  string m_expansionType;
  string m_CCconnection;
  double m_delta;
  double m_minDist;
  LocalTreeType* m_localTree;
  string m_callee = "RadialUtils";
  bool m_debug;

  public:
  RadialUtils(MPProblemType* _problem, LocalTreeType* _localTree, string _dm, string _vc, string _nf, string _CCconnection, string _expansionType,
      double _delta, double _minDist, size_t _numCCIters=0, bool _debug=false) { 

    m_problem = _problem;
    m_numCCIters = _numCCIters;
    m_dm = _dm;
    m_delta = _delta;
    m_vc = _vc;
    m_nf = _nf;
    m_CCconnection = _CCconnection;
    m_expansionType = _expansionType;
    m_minDist = _minDist;
    m_debug = _debug;

    if (_localTree == NULL)
      m_localTree = m_problem->GetRoadmap()->GetGraph();

    else 
      m_localTree = _localTree; 
  }

  // A wrapper for checking validity since sometimes the label is not set
  bool IsValid(ValidityCheckerPointer _vc, CfgType& _cfg, Environment* _env, StatClass* _stats) {
    CDInfo cdInfo;
    if (!_cfg.IsLabel("VALID")) 
      _vc->IsValid(_cfg, _env, *_stats, cdInfo, &m_callee); 
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
  void AddEdge(VID _vid1, VID _vid2, int _weight, vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights) { 

    if((_vid1 == 0 || _vid2 == 0) && stapl::get_location_id() != 0) { 
      _pendingEdges.push_back(make_pair(_vid1,_vid2));
      _pendingWeights.push_back(_weight);
      return;
    }

    //      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", _weight), WeightType("RRTExpand", _weight));
    WeightType weight("RRTExpand", _weight);
    /// m_problem->GetRoadmap()->GetGraph()->AddEdge(_vid1, _vid2, weights);
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
    globalTree->add_edge_async(_vid1, _vid2, weight);
    globalTree->add_edge_async(_vid2, _vid1, weight);
    m_localTree->add_edge(_vid1, _vid2); 
    m_localTree->add_edge(_vid2, _vid1); 
  }

  ////////////////////////////
  //  AddEdgeDebug
  ////////////////////////////
  void AddEdgeDebug(VID _vid1, VID _vid2, CfgType _cfg1, CfgType _cfg2, int _weight, vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights) { 

    if((_vid1 == 0 || _vid2 == 0) && stapl::get_location_id() != 0) { 
      _pendingEdges.push_back(make_pair(_vid1,_vid2));
      _pendingWeights.push_back(_weight);
      return;
    }

    //      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", _weight), WeightType("RRTExpand", _weight));
    WeightType weight("RRTExpand", _weight);
    /// m_problem->GetRoadmap()->GetGraph()->AddEdge(_vid1, _vid2, weights);
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
    globalTree->add_edge_async(_vid1, _vid2, weight);
    globalTree->add_edge_async(_vid2, _vid1, weight);
    m_localTree->add_edge(_vid1, _vid2); 
    m_localTree->add_edge(_vid2, _vid1); 

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

    vector<VID> kClosest;
    vector<CfgType> cfgs;

    string kcloseClockName = "kclosest time ";
    stats->StartClock(kcloseClockName);

    nf->KClosest(rdmp, _dir, 1, back_inserter(kClosest));

    stats->StopClock(kcloseClockName);

    VID nearestVID = kClosest[0];
    CfgType nearest;

    nearest = (rdmp->GetGraph()->GetCfg(nearestVID));

    ExpandTree(_currBranch, nearestVID, nearest, _dir,dummy1, dummy2);

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
    StatClass* stats = m_problem->GetStatClass();
    string callee("RadialRRT::ExpandTree");
    CDInfo cdInfo;
    int weight;  

    ExpansionType::Expansion expansion;
    vector<pair<CfgType, int> > expansionCfgs;  // this will contain all cfgs from start to goal inclusive
    expansionCfgs.push_back(make_pair(_nearest, 0));
    expansion = BlindRRTExpand<MPTraits>(m_problem, m_vc, m_dm, m_expansionType, 
        _nearest, _dir, expansionCfgs, m_delta, cdInfo, 
        env->GetPositionRes(), env->GetOrientationRes());


    if (expansion == ExpansionType::NO_EXPANSION) {  
      return 0;
    }

    CfgType& newCfg = expansionCfgs.back().first; // last cfg in the returned array is delta away from nearest
    int nodesAdded = 0;
    // If good to go, add to roadmap
    if(dm->Distance(env, newCfg, _nearest) >= m_minDist && expansion != ExpansionType::OUT_OF_BOUNDARY ) {

      // Adding Nodes
      vector<VID> expansionVIDs;
      expansionVIDs.push_back(_nearestVID);

      // we already added startCfg remember?
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {
        CfgType& cfg2 = expansionCfgs[i].first;
        VID newVID = INVALID_VID;
        // Expansion returns both valid and invalid so we can track valid edges,
        // but only add valid nodes to the tree
        if (IsValid(vc, expansionCfgs[i].first, env, stats)) { 
          newVID = AddVertex(expansionCfgs[i].first);
          _currBranch.push_back(newVID);/// branch is being replaced as local tree, remove this
        }

        expansionVIDs.push_back(newVID );

      }

      pair<WeightType, WeightType> weights;
      // Adding Edges
      int weight;
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {


        if(expansionVIDs[i-1] != INVALID_VID && expansionVIDs[i] != INVALID_VID &&
            IsValid(vc, expansionCfgs[i-1].first, env, stats) &&  
            IsValid(vc, expansionCfgs[i].first, env, stats) ) {
          weight = expansionCfgs[i].second - expansionCfgs[i-1].second; // Edge weight 

          if(!m_debug) AddEdge(expansionVIDs[i-1], expansionVIDs[i], weight, _pendingEdges, _pendingWeights);
          else AddEdgeDebug(expansionVIDs[i-1], expansionVIDs[i], expansionCfgs[i-1].first, expansionCfgs[i].first, weight, _pendingEdges, _pendingWeights);

        }

        if(expansion == ExpansionType::JUMPED) // we can only add one edge, start -> middle  
          break;
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
      exit(-1);;
    }
    //Setup MP variables
    StatClass* stats = m_problem->GetStatClass();
    RoadmapType* rdmp = m_problem->GetRoadmap();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    Environment* env = m_problem->GetEnvironment();
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


    size_t connectSwitch = 0;
    bool mapPassedEvaluation = false;
    size_t iters = 0;
    size_t failures = 0;
    size_t maxFailures = 100;

    // instead of a certain number of iterations, make it proportional to the number of CCs we have
    m_numCCIters = ccs.size() * m_numCCIters;

    while(ccs.size() > 1 && iters <= m_numCCIters && failures < maxFailures) {
      int rand1 = LRand() % ccs.size();
      cc1VID = ccs[ rand1 ].second;

      colorMap.reset();
      stapl::sequential::get_cc(*m_localTree,colorMap,cc1VID,cc1);


      if (cc1.size() == 1) {
        // PrintValue("Getting: ", cc1[0]);
        //CfgType cfg  =   (*(globalTre->distribution().container_manager().begin()->find_vertex(cc1[0]))).property();
        // CfgType cfg  =   (*(globalTre->find_vertex(cc1[0]))).property();
        //CfgType cfg  =   globalTre->GetCfg(cc1[0]);
        CfgType cfg  =   (*(m_localTree->find_vertex(cc1[0]))).property();

        if (!IsValid(vc, cfg, env, stats)) {
          failures++; 
          continue;

        }
      }
      if (m_CCconnection == "Random") {

        int rand2 = LRand() % ccs.size();
        if (rand1 == rand2) continue;
        cc2VID = ccs[ rand2 ].second; 

      } else if(m_CCconnection == "Mixed") {
        /*
           if (connectSwitch % 10 == 0) {
           cc2VID = GetClosestNodeToCentroid(cc1VID, ccs, colorMap);

           } else if (connectSwitch % 5 == 0) {
           VID randomNode = cc1[LRand() % cc1.size()];
           cc2VID = GetClosestNodeToNode(randomNode, cc1VID);

           } else if (connectSwitch % 3 == 0) {
           int rand2 = LRand() % ccs.size();
           if (rand1 == rand2) continue;
           cc2VID = ccs[ rand2 ].second; 

           } else {
           CfgType centroid = GetCentroid(cc1); 
           cc2VID = GetClosestCentroidToCentroid(centroid, cc1VID, ccs, colorMap);
           }
           connectSwitch++;        
           */
      } else {
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

        if (!IsValid(vc, cfg, env, stats)) {
          failures++;
          continue;

        }

      }
      // We got a pair of CCs, attempt to Connect them!
      pConnection->SetLocalGraph(m_localTree);
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
  //Rewrite function to take iterate through local tree instead of allVIDs vector
  void RemoveInvalidNodes(vector<VID>& _allVIDs) {

    RoadmapType* rdmp = m_problem->GetRoadmap();
    StatClass* stats = m_problem->GetStatClass();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    Environment* env = m_problem->GetEnvironment();
    CDInfo  cdInfo;
    string callee("BlindRRT::RemoveInvalidNodes");

    for (size_t i=0; i<_allVIDs.size(); i++) {
      VID vid = _allVIDs[i];
      //CfgType cfg = (*(globalTre->find_vertex(vid))).property();
      //CfgType cfg = globalTre->GetCfg(vid);
      //CfgType cfg = (*(globalTre->distribution().container_manager().begin()->find_vertex(vid))).property();
      CfgType cfg = (*(m_localTree->find_vertex(vid))).property();

      if (!IsValid(vc, cfg, env, stats)) {
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
      Environment* env = m_problem->GetEnvironment();
      vector<pair<VID,VID> > closestPair;
      nf->KClosestPairs(rdmp, _sourceCC.begin(), _sourceCC.end(), _targetCC.begin(), _targetCC.end(), 1, back_inserter(closestPair));
      if (closestPair.empty())
        return INVALID_VID;

      VID vid1 = closestPair[0].first; 
      VID vid2 = closestPair[0].second;

      CfgType cfg1 = (*(m_localTree->find_vertex(vid1))).property();
      CfgType cfg2 = (*(m_localTree->find_vertex(vid2))).property();

      return dm->Distance(env, cfg1, cfg2);
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
      Environment* env = m_problem->GetEnvironment();
      vector<VID> closestNode;
      nf->KClosest(rdmp, _sourceCC.begin(), _sourceCC.end(), targetCentroid, 1, back_inserter(closestNode));
      if (closestNode.empty())
        return INVALID_VID;

      VID closest = closestNode[0]; 
      CfgType cfg  =   (*(m_localTree->find_vertex(closest))).property();
      return dm->Distance(env, cfg, targetCentroid);

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
      Environment* env = m_problem->GetEnvironment();
      CfgType sourceCentroid = GetCentroid(_sourceCC);
      vector<VID> closestNode;
      nf->KClosest(rdmp, _targetCC.begin(), _targetCC.end(), sourceCentroid, 1, back_inserter(closestNode));
      if (closestNode.empty())
        return INVALID_VID;

      VID closest = closestNode[0]; 
      CfgType cfg  = (*(m_localTree->find_vertex(closest))).property();
      return dm->Distance(env, sourceCentroid, cfg);
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
      Environment* env = m_problem->GetEnvironment();

      return dm->Distance(env, sourceCentroid, targetCentroid);
    }

  VID
    GetClosest(VID _ccVID, vector<CCType> _ccs, string _criteria) const {
      if (m_localTree == NULL) {
        cout << "Error! Did not Set Local Tree using native view" << endl;
        exit(-1);
      }
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
        // Call different methods
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
