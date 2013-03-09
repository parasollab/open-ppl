
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
  typedef stapl::counter<stapl::default_timer> STAPLTimer;
  typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
  typedef pair<size_t, VID> CCType;
  typedef typename stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
  typedef typename RoadmapViewType::view_container_type LocalTreeType;

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

  public:
  RadialUtils(MPProblemType* _problem, string _dm, string _vc, string _nf, string _CCconnection, string _expansionType,
      double _delta, double _minDist, size_t _numCCIters=0) {

    m_problem = _problem;
    m_numCCIters = _numCCIters;
    m_dm = _dm;
    m_delta = _delta;
    m_vc = _vc;
    m_nf = _nf;
    m_CCconnection = _CCconnection;
    m_expansionType = _expansionType;
    m_minDist = _minDist;

  }

  void SetLocalTree(LocalTreeType& _localTree) { m_localTree = &_localTree; }
  
  // A wrapper for checking validity since sometimes the label is not set
  bool IsValid(ValidityCheckerPointer _vc, CfgType& _cfg, Environment* _env, StatClass* _stats) {
    CDInfo cdInfo;
    if (!_cfg.IsLabel("VALID")) 
      _vc->IsValid(_cfg, _env, *_stats, cdInfo, &m_callee); 
    return _cfg.GetLabel("VALID");
  }

  inline VID AddVertex(CfgType _cfg) const {
    VID newVID = m_problem->GetRoadmap()->GetGraph()->add_vertex(_cfg);
    //  _localTree->add_vertex(newVID, _cfg); 
    // VDAddNode(_cfg);
    return newVID;
  }

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
    globalTree->add_edge(_vid1, _vid2, weight);
    globalTree->add_edge(_vid2, _vid1, weight);
    //  _localTree->add_edge(_vid1, _vid2, _weight); 
    //  _localTree->add_edge(_vid2, _vid1, _weight); 

  }


  int ExpandTree(vector<VID>& _currBranch, VID _nearestVID, CfgType& _nearest, CfgType& _dir, 
      vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights, STAPLTimer& expansionClk, STAPLTimer& process)  {
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
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
      expansionClk.start();
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {
        CfgType& cfg2 = expansionCfgs[i].first;
        VID newVID = AddVertex(expansionCfgs[i].first);
        expansionVIDs.push_back(newVID );
        _currBranch.push_back(newVID);
      }

      expansionClk.stop();
      pair<WeightType, WeightType> weights;
      // Adding Edges
      int weight;
      process.start();
      for(size_t i=1; i<expansionCfgs.size(); i++ ) {


        if(IsValid(vc, expansionCfgs[i-1].first, env, stats) &&  
            IsValid(vc, expansionCfgs[i].first, env, stats) ) {
          weight = expansionCfgs[i].second - expansionCfgs[i-1].second; // Edge weight 
          AddEdge(expansionVIDs[i-1], expansionVIDs[i], weight, _pendingEdges, _pendingWeights);
        }

        if(expansion == ExpansionType::JUMPED) // we can only add one edge, start -> middle  
          break;
      }
      process.stop();

      nodesAdded=  expansionCfgs.size() - 1;    // substract one cause the start already belonged to the tree
    }

    return nodesAdded;

  }
  // TODO: Debug timer if there is need for using it!
  void ConnectCCs( /*vector<STAPLTimer>& _timer */) {
    if (m_localTree == NULL) {
      cout << "Error! Did not Set Local Tree using native view" << endl;
      exit(-1);;
    }
    //Setup MP variables
    StatClass* stats = m_problem->GetStatClass();
    RoadmapType* rdmp = m_problem->GetRoadmap();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
    Environment* env = m_problem->GetEnvironment();
    DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
    ConnectorPointer pConnection = m_problem->GetConnector("RRTConnect");
    CDInfo cdInfo;
    string callee("BlindRRT::RemoveInvalidNodes");

    stringstream clockName; clockName << "Component Connection";
    stats->StartClock(clockName.str());

//    _timer[0].start();
    ColorMap colorMap;
    vector< CCType > ccs;
    colorMap.reset();
    stapl::sequential::get_cc_stats(*m_localTree, colorMap, ccs);      
  //  _timer[0].stop();

    if(ccs.size()==1) return;

    vector<VID> cc1;
    vector<VID> cc2;
    VID cc1VID; 
    VID cc2VID; 

    bool alt = false;
    bool mapPassedEvaluation = false;
    size_t iters = 0;
    size_t failures = 0;
    size_t maxFailures = 100;
    while(ccs.size() > 1 && iters <= m_numCCIters && failures < maxFailures) {
      int rand1 = LRand() % ccs.size();
      // always expand from the root CC
      cc1VID = ccs[ rand1 ].second; 
      stapl::sequential::get_cc(*m_localTree,colorMap,cc1VID,cc1);
      if (cc1.size() == 1) {
        PrintValue("Getting: ", cc1[0]);
        CfgType cfg  =   (*(globalTree->distribution().container_manager().begin()->find_vertex(cc1[0]))).property();

        if (!IsValid(vc, cfg, env, stats)) {
          failures++; 
          continue;
        
        }
      }
      if (m_CCconnection == "Random") {

        int rand2 = LRand() % ccs.size();
        if (rand1 == rand2) continue;
        cc2VID = ccs[ rand2 ].second; 

      } else if(m_CCconnection == "ClosestNode") {

        VID randomNode = cc1[LRand() % cc1.size()];

    //    _timer[1].start();
        cc2VID = GetClosestCCByNode(randomNode, cc1VID);
    //    _timer[1].stop();

      } else if (m_CCconnection == "ClosestCC") {
        CfgType centroid = GetCentroid(globalTree,cc1);
        cc2VID = GetClosestCCByCentroid(centroid, cc1VID, ccs, colorMap);
        

      } else if(m_CCconnection == "Mixed") {
        if(alt) {
          int rand2 = LRand() % ccs.size();
          if (rand1 == rand2) continue;
          cc2VID = ccs[ rand2 ].second; 

        } else {
          VID randomNode = cc1[LRand() % cc1.size()];
          cc2VID = GetClosestCCByNode(randomNode, cc1VID);
        }
        alt = !alt;

      } else {
        cout << "Unknown CC connection type: " << m_CCconnection << endl;
        exit(-1);

      }
      stapl::sequential::get_cc(*m_localTree,colorMap,cc2VID,cc2);

      // Maybe this is an invalid node, don't use it
      if (cc2.size() == 1) {

        CfgType cfg  =   (*(globalTree->distribution().container_manager().begin()->find_vertex(cc2[0]))).property();

        if (!IsValid(vc, cfg, env, stats)) {
          failures++;
          continue;
          
        }

      }
      // We got a pair of CCs, attempt to Connect them!
//      _timer[2].start();
      pConnection->Connect(rdmp, *stats, colorMap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end()) ;
 //     _timer[2].stop();

      iters++;
      colorMap.reset();
      stapl::sequential::get_cc_stats(*m_localTree, colorMap, ccs);      

    }

    stats->StopClock(clockName.str());

  }


  void RemoveInvalidNodes(vector<VID>& _allVIDs) {

    RoadmapType* rdmp = m_problem->GetRoadmap();
    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
    StatClass* stats = m_problem->GetStatClass();
    ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
    Environment* env = m_problem->GetEnvironment();
    CDInfo  cdInfo;
    string callee("BlindRRT::RemoveInvalidNodes");

    for (size_t i=0; i<_allVIDs.size(); i++) {
      VID vid = _allVIDs[i];
      CfgType cfg = (*(globalTree->distribution().container_manager().begin()->find_vertex(vid))).property();

      if (!IsValid(vc, cfg, env, stats))
        rdmp->GetGraph()->delete_vertex(_allVIDs[i]); 
      //VDRemoveNode(cfg); 
    }
  }





  VID GetClosestCCByNode(VID _node, VID _nodeCCVID) const {
    if (m_localTree == NULL) {
      cout << "Error! Did not Set Local Tree using native view" << endl;
      exit(-1);
    }

    RoadmapType* rdmp = m_problem->GetRoadmap();
    NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);

    ColorMap colorMap;
    vector< pair<size_t,VID> > ccs;
    stapl::sequential::get_cc_stats(*m_localTree,colorMap,ccs);

    typedef typename vector<pair<size_t, VID> >::iterator CCSIT;

    // Key = nodeVID, Value = CCVID
    // For easy retrieval of the CC once the closest node is found
    map<VID, VID> nodesAndCCs;
    typedef typename vector<pair<size_t, VID> >::iterator CCSIT;
    vector<VID> closestNodesOtherCCs;
    //find closest VID from other CCS
    for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){

      if(ccsit->second == _nodeCCVID)
        continue;

      vector<VID> cc;
      stapl::sequential::get_cc(*m_localTree,colorMap,ccsit->second,cc);
      colorMap.reset();
      vector<VID> closest;
      nf->KClosest(rdmp, cc.begin(), cc.end(), _node, 1, back_inserter(closest));
      if (closest.size() != 0) { 
        closestNodesOtherCCs.push_back(closest[0]);
        nodesAndCCs[closest[0]] = ccsit->second; 
      }
    }

    //find closest VID from other CCS reps
    vector<VID> closestNode;
    nf->KClosest(rdmp, closestNodesOtherCCs.begin(), 
        closestNodesOtherCCs.end(), _node, 1, back_inserter(closestNode));

    VID closestCC = nodesAndCCs[ closestNode[0] ];

    return closestCC;
  }

  // returns the VID of the centroid from the vec _ccs that is closest to _centroid
  VID GetClosestCCByCentroid(CfgType _centroid, VID _ccVID, vector<pair<size_t, VID> > _ccs, ColorMap _colorMap) const {
    if (m_localTree == NULL) {
      cout << "Error! Did not Set Local Tree using native view" << endl;
      exit(-1);
    }
    DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
    Environment* env = m_problem->GetEnvironment();

    GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
    
    vector<VID> cc;

    double currMinDist = MAX_DBL;
    VID currMinVID = INVALID_VID;

    for (int i=0; i<_ccs.size(); i++) {
      // TODO Cesar fix STAPL 
      if(_ccs[i].second == _ccVID)
        continue;
      
      stapl::sequential::get_cc(*m_localTree,_colorMap,_ccs[i].second,cc);
      CfgType otherCentroid = GetCentroid(globalTree,cc);
      double dist = dm->Distance(env, _centroid, otherCentroid);
      if (dist < currMinDist) {
        currMinDist = dist;
        currMinVID = _ccs[i].second;
      }

      cc.clear();
    }
    return currMinVID;
  }



};
