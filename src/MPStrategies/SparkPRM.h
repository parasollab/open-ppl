#ifndef SPARK_PRM_H_
#define SPARK_PRM_H_

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief Constructs RRTs within narrow passages to quickly plan through them.
/// @tparam MPTraits Motion planning universe
///
/// Constructs RRTs within narrow passages to quickly plan through them.
///
/// \internal This strategy is configured for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits, template<typename> class Strategy>
class SparkPRM : public Strategy<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename GraphType::const_vertex_iterator CVI;

    SparkPRM(size_t _maxNPCCSize = 3, size_t _initSamples = 30,
        size_t _maxRRTSize = 100, size_t _attemptRatio = 10,
        size_t _trimDepth = 0, bool _checkImportant = true,
        bool _checkEdgeCases = true, bool _trimAll = true,
        bool _biasConnect = false, bool _checkStartGoal = true,
        double _delta = 10, double _minDist = 0.001, double _growthFocus = 0.05,
        string _dmLabel = "", string _nfLabel = "", string _nfVertexLabel = "",
        string _vcLabel = "", string _cLabel = "", string _eLabel = "",
        bool _rrtDebug = false);
    SparkPRM(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os);

    // Performs a narrow passage test and then constructs an RRT if successful.
    // Returns true if start and goal connected.
    virtual bool CheckNarrowPassageSample(VID _vid);

  private:
    // Helper method to remove CCs connected to the RRT from the centroids list
    // and to update notRRT if needed
    bool UpdateCentroids(RoadmapType* _centroidRdmp, vector<VID>& _notRRT,
        VID _root);

  protected:

    // Checks if vertex was unable to connect to any other major CCs.
    // The cutoff CC size is given by m_maxNPCCSize. Call this method after
    // connecting the vertex to be tested.
    bool NarrowPassage(VID _root);

    // Builds an RRT and adds it to the map. Returns true if start and goal
    // connected.
    bool ConstructRRT(VID _root);

    // Finds the centroid of each CC other than the RRT itself, and adds
    // vertices to the notRRT list
    void ComputeCentroids(RoadmapType* _centroidRdmp, vector<VID>& _notRRT,
        VID _root);

    // Connects an RRT vertex to the rest of the map. Connection can be biased
    // if desired.
    void ConnectVertex(RoadmapType* _centroidRdmp, vector<VID>& _notRRT,
        VID _recentVID);

    // Trims the RRT. Returns the number of deleted vertices.
    int TrimRRT(vector<VID>& _rrt, vector<VID>& _important, int _connectedCCs);

    // Attempts to expand the RRT to the given direction
    VID ExpandTree(CfgType& _dir, vector<VID>& _rrt, vector<VID>& _important);

    // Pick a big enough random CC (not the RRT's CC), then pick a random
    // vertex in it
    CfgType GoalBiasedDirection(VID _rrt);
    // Selects a random direction for growth
    CfgType SelectDirection();

    size_t m_maxNPCCSize;   // The maximum size of CC that we consider to be
                            // within a narrow passage
    size_t m_initSamples;   // Number of initial samples before RRT strategies
                            // start
    size_t m_maxRRTSize;    // Maximum number of vertices in an RRT before we
                            // give up and end growth
    size_t m_attemptRatio;  // How many attempts per RRT vertex
    size_t m_trimDepth;     // Depth of tree pruning (0 to keep entire tree)

    bool m_checkImportant;  // Check for connection to important nodes?
    bool m_checkEdgeCases;  // Check if the RRT is an edge case?
    bool m_trimAll;         // Trim the RRTs that only connect to one CC?
    bool m_biasConnect;     // Connect RRT vertices only to the closest CC?
    bool m_checkStartGoal;  // Perform narrow passage test on start and goal?

    double m_delta;         // Maximum distance to expand
    double m_minDist;       // Minimum distance to expand
    double m_growthFocus;   // How often to direct growth toward goal
    string m_dmLabel;       // Distance metric
    string m_nfLabel;       // Neighborhood finder
    string m_nfVertexLabel; // Neighborhood finder for ConnectVertex step, k=1
    string m_vcLabel;       // Validity checker
    string m_cLabel;        // Node connector
    string m_eLabel;        // Extender Method

    bool m_rrtDebug;        // like m_debug but for the rrt stuff only. TODO:
                            // get rid of this (but imo reading the debug output
                            // is really hard)
};

template<class MPTraits, template<typename> class Strategy>
SparkPRM<MPTraits, Strategy>::
SparkPRM(size_t _maxNPCCSize, size_t _initSamples,
    size_t _maxRRTSize, size_t _attemptRatio, size_t _trimDepth,
    bool _checkImportant, bool _checkEdgeCases, bool _trimAll,
    bool _biasConnect, bool _checkStartGoal,
    double _delta, double _minDist, double _growthFocus,
    string _dmLabel, string _nfLabel, string _nfVertexLabel,
    string _vcLabel, string _cLabel, string _eLabel,
    bool _rrtDebug) :
  m_maxNPCCSize(_maxNPCCSize), m_initSamples(_initSamples),
  m_maxRRTSize(_maxRRTSize), m_attemptRatio(_attemptRatio),
  m_trimDepth(_trimDepth), m_checkImportant(_checkImportant),
  m_checkEdgeCases(_checkEdgeCases), m_trimAll(_trimAll),
  m_biasConnect(_biasConnect), m_checkStartGoal(_checkStartGoal),
  m_delta(_delta), m_minDist(_minDist), m_growthFocus(_growthFocus),
  m_dmLabel(_dmLabel), m_nfLabel(_nfLabel), m_nfVertexLabel(_nfVertexLabel),
  m_vcLabel(_vcLabel), m_cLabel(_cLabel), m_eLabel(_eLabel),
  m_rrtDebug(_rrtDebug) {
    this->m_name += "WithRRT";
  }

template<class MPTraits, template<typename> class Strategy>
SparkPRM<MPTraits, Strategy>::
SparkPRM(MPProblemType* _problem, XMLNode& _node) :
  Strategy<MPTraits>(_problem, _node) {
    this->m_name += "WithRRT";
    ParseXML(_node);
  }

template<class MPTraits, template<typename> class Strategy>
void
SparkPRM<MPTraits, Strategy>::
ParseXML(XMLNode& _node) {

  m_maxNPCCSize = _node.Read("maxNPCCSize", false, 1, 0, MAX_INT,
      "Maximum size of a CC within a narrow passage");
  m_initSamples = _node.Read("initSamples", false, 50, 0, MAX_INT,
      "Initial samples before RRT strategy starts");
  m_maxRRTSize = _node.Read("maxRRTSize", false, 100, 0, MAX_INT,
      "Maximum number of vertices in an RRT");
  m_attemptRatio = _node.Read("attemptRatio", false, 10, 1,
      MAX_INT, "Maximum number of attempts per node");
  m_trimDepth = _node.Read("trimDepth", false, 0, 0, MAX_INT,
      "The depth at which the RRT is trimmed");

  m_checkImportant = _node.Read("checkImportant", false, true,
      "Check for connection to Important nodes?");
  m_checkEdgeCases = _node.Read("checkEdgeCases", false, true,
      "Check for edge cases?");
  m_trimAll = _node.Read("trimAll", false, true,
      "Trim the RRTs thatonly connect to one CC?");
  m_biasConnect = _node.Read("biasConnect", false, false,
      "Connect RRT vertices only to the closest CC?");
  m_checkStartGoal = _node.Read("checkStartGoal", false, true,
      "Build RRTs from the start and goal?");

  m_delta = _node.Read("delta", false, 1.0, 0.0, MAX_DBL, "Delta Distance");
  m_minDist = _node.Read("minDist", false, 0.0, 0.0, MAX_DBL,
      "Minimum Distance");
  m_growthFocus = _node.Read("growthFocus", false, 0.0, 0.0, 1.0,
      "#GeneratedTowardsGoal/#Generated");
  m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_nfVertexLabel = _node.Read("nfVertexLabel", true, "Nearest",
      "Neighborhood Finder for ConnectVertex step, k=1");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_cLabel = _node.Read("cLabel", true, "", "Node Connection Method");
  m_eLabel = _node.Read("eLabel", true, "", "Extender Method");

  m_rrtDebug = _node.Read("rrtDebug", false, false, "Debug for RRT stuff");
}

template<class MPTraits, template<typename> class Strategy>
void
SparkPRM<MPTraits, Strategy>::
Print(ostream& _os) {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tmaxNPCCSize = " << m_maxNPCCSize;
  _os << "\n\tinitSamples = " << m_initSamples;
  _os << "\n\tmaxRRTSize = " << m_maxRRTSize;
  _os << "\n\tattemptRatio = " << m_attemptRatio;
  _os << "\n\ttrimDepth = " << m_trimDepth;
  _os << "\n\tcheckImportant = " << m_checkImportant;
  _os << "\n\tcheckEdgeCases = " << m_checkEdgeCases;
  _os << "\n\ttrimDepth = " << m_trimDepth;
  _os << "\n\tbiasConnect = " << m_biasConnect;
  _os << "\n\tcheckStartGoal = " << m_checkStartGoal;
  _os << "\n\tdelta = " << m_delta;
  _os << "\n\tminDist = " << m_minDist;
  _os << "\n\tgrowthFocus = " << m_growthFocus;
  _os << "\n\tdmLabel = " << m_dmLabel;
  _os << "\n\tnfLabel = " << m_nfLabel;
  _os << "\n\tnfVertexLabel = " << m_nfVertexLabel;
  _os << "\n\tvcLabel = " << m_vcLabel;
  _os << "\n\tcLabel = " << m_cLabel;
  _os << "\n\teLabel = " << m_eLabel;
  _os << "\n\trrtDebug = " << m_rrtDebug;
  _os << endl;
}

// Performs a narrow passage test and then constructs an RRT if successful.
// Returns true if start and goal connected.
template<class MPTraits, template<typename> class Strategy>
bool
SparkPRM<MPTraits, Strategy>::
CheckNarrowPassageSample(VID _vid) {
  GraphType* graph = this->GetRoadmap()->GetGraph();

  // As soon as sufficient initial samples are created, grow from the start and
  // goal
  // TODO NOTE: This step assumes that the start and goal are VIDs 0 and 1!
  if(m_checkStartGoal && graph->get_num_vertices() >= m_initSamples) {
    stapl::sequential::vector_property_map<GraphType, size_t> cMap;
    m_checkStartGoal = false;
    CheckNarrowPassageSample(0);
    if(is_same_cc(*graph, cMap, 0, 1))
      return true;
    CheckNarrowPassageSample(1);
    cMap.reset();
    if(is_same_cc(*graph, cMap, 0, 1))
      return true;
  }

  if(NarrowPassage(_vid))
    return ConstructRRT(_vid);
  return false;
}

// Checks if vertex was unable to connect to any other major CCs, and build an
// RRT if so. The cutoff CC size is given by m_maxNPCCSize. Call this method
// after connecting the vertex to be tested.
template<class MPTraits, template<typename> class Strategy>
bool
SparkPRM<MPTraits, Strategy>::
NarrowPassage(VID _vid) {

  // Do not start RRTs until sufficient initial samples created
  if(this->GetRoadmap()->GetGraph()->get_num_vertices() < m_initSamples)
    return false;

  if(m_maxNPCCSize == 1)
    return this->GetRoadmap()->GetGraph()->get_degree(_vid) == 0;

  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  StatClass* stats = this->GetStatClass();
  stats->StartClock("RRT: Narrow Passage");
  vector<VID> cc;
  size_t ccSize = get_cc(*(this->GetRoadmap()->GetGraph()), cMap, _vid, cc);
  stats->StopClock("RRT: Narrow Passage");
  return ccSize <= m_maxNPCCSize;
}

// Builds an RRT and adds it to the map. Returns true if start and goal
// connected.
template<class MPTraits, template<typename> class Strategy>
bool
SparkPRM<MPTraits, Strategy>::
ConstructRRT(VID _root) {

  GraphType* graph = this->GetRoadmap()->GetGraph();
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;

  if(this->m_rrtDebug)
    cout << "\nConstructing an RRT! VID = " << _root << endl;

  StatClass* stats = this->GetStatClass();
  stats->StartClock("Total RRT");
  int initialCD = stats->GetIsCollTotal();

  bool stopGrowth = false; // Flag to stop RRT growth
  size_t attempts = 0;     // Number of attempts of expanding the RRT
  size_t connectedCCs = 0; // How many old CCs the RRT connected to
  size_t oldCCSize = 0;    // The total size of all old CCs the RRT connects to,
                           // running total as they are found

  VID recentVID;
  vector<VID> rrt, important, cc;
  vector<VID> notRRT; // Used only if m_biasConnect is false (connect to
                      // everything outside the RRT's CC)
  RoadmapType centroidRdmp;

  rrt.push_back(_root);

  // Find the centroids of the CCs
  ComputeCentroids(&centroidRdmp, notRRT, _root);

  // Build the RRT until we need to stop
  while(!stopGrowth) {

    // Find growth direction
    double randomRatio = DRand();
    CfgType dir;
    if(randomRatio < m_growthFocus)
      dir = this->GoalBiasedDirection(_root);
    else
      dir = this->SelectDirection();

    // Grow towards the direction
    attempts++;
    stats->StartClock("RRT: ExpandTree");
    recentVID = this->ExpandTree(dir, rrt, important);
    stats->StopClock("RRT: ExpandTree");

    // Stop growth if RRT becomes too large, or too many attempts taken
    if(rrt.size() >= m_maxRRTSize || attempts >= m_attemptRatio*m_maxRRTSize)
      stopGrowth = true;

    if(recentVID == INVALID_VID)
      continue;

    // rrt should be strictly increasing. If the latest VID in the RRT is not
    // numerically larger than the others, the vertex was already in the graph
    // and should not be in rrt.
    if(rrt[rrt.size()-1] <= rrt[rrt.size()-2]) {
      if(this->m_rrtDebug)
        cout << "  Vertex VID = " << rrt[rrt.size()-1] << endl;
      // The vertex might have been in a separate CC that is now connected to
      // the RRT
      if(UpdateCentroids(&centroidRdmp, notRRT, _root)) {

        // Is the new CC is big enough to count as outside the narrow passage?
        cMap.reset();
        size_t totalSize = get_cc(*graph, cMap, _root, cc);
        if(rrt.size() + oldCCSize + m_maxNPCCSize <= totalSize) {

          connectedCCs++;
          VID importantVID =
              graph->GetVertex(rrt[rrt.size()-1]).GetStat("Parent");
          important.push_back(importantVID);
          if(this->m_rrtDebug)
            cout << "New Important VID: " << importantVID << endl;

          // If the RRT connects too quickly, it's probably an edge case, don't
          // waste time on it
          if(m_checkEdgeCases && rrt.size() <= 3)
            stopGrowth = true;
        }

        // Update oldCCSize
        oldCCSize = totalSize - (rrt.size()-1);

        // Stop growth when RRT connects to two different old CCs
        if(connectedCCs >= 2)
          stopGrowth = true;
      }
      rrt.erase(rrt.end() - 1);
      continue;
    }

    if(this->m_rrtDebug)
      cout << "  RRT Size = " << rrt.size() << ". Attempts = " << attempts
        << ". VID = " << recentVID << "\n    Cfg: "
        << graph->GetVertex(recentVID) << endl;

    // Stop growth if RRT becomes too large
    if(rrt.size() >= m_maxRRTSize)
      stopGrowth = true;

    // Find number of edges before connecting RRT to other CCs
    size_t edges = graph->get_num_edges();

    // Try to connect the recent vertex to the rest of the map
    ConnectVertex(&centroidRdmp, notRRT, recentVID);

    // Check if RRT was connected to other CCs (an edge was created)
    if(edges < graph->get_num_edges()) {

      // Find totalSize (the size of the RRT's CC, including the RRT itself)
      cMap.reset();
      size_t totalSize = get_cc(*graph, cMap, recentVID, cc);

      if(this->m_rrtDebug)
        cout << "  Total size = " << totalSize << ", rrt.size() = "
          << rrt.size() << "\n  Connected to a CC of size "
          << (totalSize - rrt.size() - oldCCSize) << endl;

      // Is the new CC is big enough to count as outside the narrow passage?
      if(rrt.size() + oldCCSize + m_maxNPCCSize <= totalSize) {
        connectedCCs++;
        important.push_back(recentVID);
        if(this->m_rrtDebug) cout << "New Important VID: " << recentVID << endl;

        // If the RRT connects too quickly, it's probably an edge case, don't
        // waste time on it
        if(m_checkEdgeCases && rrt.size() <= 3)
          stopGrowth = true;
      }

      // RRT connected to other CCs, so update centroids
      UpdateCentroids(&centroidRdmp, notRRT, _root);

      // Update oldCCSize (number of vertices in the RRT's CC but not part of
      // the RRT)
      oldCCSize = totalSize - rrt.size();

      // Stop growth when RRT connects to two different old CCs
      if(connectedCCs >= 2)
        stopGrowth = true;

      // Only one old CC is needed if root is start or goal
      if((_root == 0 || _root == 1) && connectedCCs >= 1)
        stopGrowth = true;
    }
  }

  // Trim the RRT, save the number of vertices deleted
  int numDeleted = TrimRRT(rrt, important, connectedCCs);

  if(rrt.size() > 3) {
    stats->IncRRTStat("Number of RRTs");
    stats->IncRRTStat("RRT Vertices", rrt.size()-numDeleted);
    stats->IncRRTStat("Deleted Vertices", numDeleted);
  }

  stats->StopClock("Total RRT");
  stats->IncRRTStat("RRT CD Calls", stats->GetIsCollTotal() - initialCD);
  if(this->m_rrtDebug)
    cout << "End constructing RRT. Size = " << (rrt.size()-numDeleted)
         << ", connected CCs = " << connectedCCs << endl;

  cMap.reset();
  // TODO NOTE: This step assumes that the start and goal are VIDs 0 and 1!
  return (connectedCCs == 2 && is_same_cc(*graph, cMap, 0, 1));
}

// Finds the centroid of each CC other than the RRT itself, and adds vertices
// to the notRRT list
template<class MPTraits, template<typename> class Strategy>
void
SparkPRM<MPTraits, Strategy>::
ComputeCentroids(RoadmapType* _centroidRdmp, vector<VID>& _notRRT, VID _root) {

  StatClass* stats = this->GetStatClass();
  stats->StartClock("RRT: ComputeCentroids");
  GraphType* graph = this->GetRoadmap()->GetGraph();
  GraphType* centroidGraph = _centroidRdmp->GetGraph();
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  vector<pair<size_t, VID> > allCCs;
  vector<VID> cc;
  get_cc_stats(*graph, cMap, allCCs);

  // Get centroids, remove the RRT's CC
  ComputeCCCentroidGraph(graph, centroidGraph);
  for(CVI it = centroidGraph->begin(); it != centroidGraph->end(); it++) {
    cMap.reset();
    if(is_same_cc(*graph, cMap, _root,
        (VID)((CfgType)it->property()).GetStat("ccVID"))) {
      centroidGraph->delete_vertex(it->descriptor());
      break;
    }
  }

  // Compute notRRT
  for(size_t i = 0; i < allCCs.size(); i++) {
    // Skip if the same CC as the RRT root
    cMap.reset();
    if(is_same_cc(*graph, cMap, allCCs[i].second, _root))
      continue;
    cMap.reset();
    get_cc(*graph, cMap, allCCs[i].second, cc);
    if(!m_biasConnect)
      _notRRT.insert(_notRRT.end(), cc.begin(), cc.end());
  }
  stats->StopClock("RRT: ComputeCentroids");
}

// Connects an RRT vertex to the rest of the map. Connection can be biased if
// desired.
template<class MPTraits, template<typename> class Strategy>
void
SparkPRM<MPTraits, Strategy>::
ConnectVertex(RoadmapType* _centroidRdmp, vector<VID>& _notRRT, VID _recentVID) {

  StatClass* stats = this->GetStatClass();
  stats->StartClock("RRT: ConnectVertex");
  ConnectorPointer connector = this->GetConnector(m_cLabel);
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  GraphType* centroidGraph = _centroidRdmp->GetGraph();

  // Try to connect RRT to other CCs. If code is not buggy (haha),
  // CheckIfSameCC is redundant and should be "false"
  if(m_biasConnect) {
    // Connect to the closest CC. Skip connection if there are no other CCs
    if(centroidGraph->get_num_vertices() != 0) {

      stats->StartClock("RRT: BiasConnect");
      NeighborhoodFinderPointer nf =
          this->GetNeighborhoodFinder(m_nfVertexLabel);
      GraphType* graph = this->GetRoadmap()->GetGraph();
      vector<VID> cc;
      vector<pair<VID, double> > closest;
      CfgType recentCfg = graph->GetVertex(_recentVID);

      nf->FindNeighbors(_centroidRdmp, recentCfg, back_inserter(closest));
      get_cc(*graph, cMap,
          centroidGraph->GetVertex(closest[0].first).GetStat("ccVID"), cc);

      stats->StopClock("RRT: BiasConnect");
      connector->Connect(this->GetRoadmap(), _recentVID, cc.begin(), cc.end(), false);
    }
  }
  else {
    // If not biasing connection, connect to everything else
    connector->Connect(this->GetRoadmap(), _recentVID,
        _notRRT.begin(), _notRRT.end(), false);
  }
  stats->StopClock("RRT: ConnectVertex");
}

// Trims the RRT. Returns the number of deleted vertices.
template<class MPTraits, template<typename> class Strategy>
int
SparkPRM<MPTraits, Strategy>::
TrimRRT(vector<VID>& _rrt, vector<VID>& _important, int _connectedCCs) {

  //TODO: Reimplement this with STAPL stuff
  // Sorry Jory, I just never got around to doing that

  StatClass* stats = this->GetStatClass();
  stats->StartClock("RRT: TrimRRT");
  GraphType* graph = this->GetRoadmap()->GetGraph();
  set<VID> trimmed;

  int numDeleted = 0;
  if(m_trimDepth > 0) {
    // If trimming the case where connectedCCs = 1, then use path from root to
    // first important vertex
    if(m_trimAll && _connectedCCs == 1)
      _important.push_back(_rrt[0]);

    if(_connectedCCs == 2 || (m_trimAll && _connectedCCs == 1)) {
      vector<VID> path;
      find_path_dijkstra(*graph, _important[0], _important[1], path,
          WeightType::MaxWeight());
      stats->IncGOStat("Graph Search");

      queue<pair<VID, size_t> > q;

      // Put each vertex along the shortest path in the queue, giving each a
      // depth of 1
      for(size_t i = 0; i < path.size(); i++)
        q.push(make_pair(path[i], 1));

      // BFS
      while(!q.empty()) {

        VID cur = q.front().first;
        size_t depth = q.front().second;
        q.pop();

        // If element already existed in the set or the trim depth is reached,
        // continue
        if(!trimmed.insert(cur).second || depth == m_trimDepth)
          continue;

        // Put neighbors in queue with depth+1
        vector<VID> adj;
        graph->get_successors(cur, adj);
        for(size_t i = 0; i < adj.size(); i++)
          q.push(make_pair(adj[i], depth+1));
      }

      // Don't delete the root, to preserve benefits of whatever sampling
      // technique used
      trimmed.insert(_rrt[0]);

      if(this->m_rrtDebug) {
        cout << "\n\n *** Trimming RRT ***\n\nRRT:";
        for(size_t i = 0; i < _rrt.size(); i++)
          cout << " " << _rrt[i];
        cout << "\nImportant VIDs: " << _important[0] << " " << _important[1]
             << "\nPath VIDs:";
        for(size_t i = 0; i < path.size(); i++)
          cout << " " << path[i];
        cout << "\ntrimmed:";
        for(typename set<VID>::iterator it = trimmed.begin();
            it != trimmed.end(); it++)
          cout << " " << *it;
        cout << "\nDeleted VIDs:";
      }

      // Delete everything not saved in the set
      for(typename vector<VID>::iterator it = _rrt.begin(); it != _rrt.end();
          it++) {
        if(trimmed.find(*it) == trimmed.end()) {
          if(this->m_rrtDebug)
            cout << " " << *it;
          numDeleted++;
          graph->delete_vertex(*it);
        }
      }

      if(this->m_rrtDebug)
        cout << "\n\n *** End ***\n\n";
    }
  }

  stats->StopClock("RRT: TrimRRT");
  return numDeleted;
}

// Helper method to remove CCs connected to the RRT from the centroids list and
// to update notRRT if needed
template<class MPTraits, template<typename> class Strategy>
bool
SparkPRM<MPTraits, Strategy>::
UpdateCentroids(RoadmapType* _centroidRdmp, vector<VID>& _notRRT, VID _root) {
  StatClass* stats = this->GetStatClass();
  stats->StartClock("RRT: UpdateCentroids");
  GraphType* graph = this->GetRoadmap()->GetGraph();
  GraphType* centroidGraph = _centroidRdmp->GetGraph();
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  bool needUpdate = false;

  // Remove any CCs connected to the RRT
#ifdef VIZMO
  for(CVI it = centroidGraph->begin(); it != centroidGraph->end();) {
    cMap.reset();
    if(is_same_cc(*graph, cMap, _root,
        (VID)((CfgType)it->property()).GetStat("ccVID"))) {
      centroidGraph->delete_vertex(it->descriptor());
      it = centroidGraph->begin();
      needUpdate = true;
    }
    else
      it++;
  }
#else
  for(CVI it = centroidGraph->begin(); it != centroidGraph->end(); it++) {
    cMap.reset();
    if(is_same_cc(*graph, cMap, _root,
        (VID)((CfgType)it->property()).GetStat("ccVID"))) {
      centroidGraph->delete_vertex(it->descriptor());
      it--;
      needUpdate = true;
    }
  }
#endif

  // Reconstruct the notRRT list if needed
  if(needUpdate && !m_biasConnect) {
    vector<VID> cc;
    _notRRT.clear();
    for(CVI it = centroidGraph->begin(); it != centroidGraph->end(); it++) {
      cMap.reset();
      get_cc(*graph, cMap, (VID)((CfgType)it->property()).GetStat("ccVID"), cc);
      _notRRT.insert(_notRRT.end(), cc.begin(), cc.end());
    }
  }
  stats->StopClock("RRT: UpdateCentroids");
  return needUpdate;
}

// Pick a big enough random CC (not the RRT's CC), then pick a random vertex in
// it
template<class MPTraits, template<typename> class Strategy>
typename MPTraits::CfgType
SparkPRM<MPTraits, Strategy>::
GoalBiasedDirection(VID _rrt) {

  StatClass* stats = this->GetStatClass();
  stats->StartClock("Biased Direction");
  GraphType* graph = this->GetRoadmap()->GetGraph();
  stapl::sequential::vector_property_map<GraphType, size_t> cMap;
  vector<pair<size_t, VID> > allCCs;
  vector<VID> randCC;
  get_cc_stats(*graph, cMap, allCCs);
  cMap.reset();
  size_t randCCIndex = LRand() % allCCs.size();

  // Don't pick a CC that is too small or that contains the RRT
  while(allCCs[randCCIndex].first < m_maxNPCCSize ||
      allCCs[randCCIndex].second == _rrt ||
      is_same_cc(*graph, cMap, _rrt, allCCs[randCCIndex].second)) {
    cMap.reset();
    allCCs.erase(allCCs.begin()+randCCIndex);
    // All CCs are undesirable, return a random direction
    if(allCCs.size() == 0)
      return SelectDirection();
    randCCIndex = LRand() % allCCs.size();
  }

  // Pick a random vertex within the random CC
  get_cc(*graph, cMap, allCCs[randCCIndex].second, randCC);
  size_t randIndex = LRand() % randCC.size();
  stats->StopClock("Biased Direction");
  return graph->GetVertex(randCC[randIndex]);
}

// Selects a random direction for growth
template<class MPTraits, template<typename> class Strategy>
typename MPTraits::CfgType
SparkPRM<MPTraits, Strategy>::
SelectDirection() {
  Environment* env = this->GetEnvironment();
  CfgType dir;
  dir.GetRandomCfg(env);
  return dir;
}

// Attempts to expand the RRT to the given direction
template<class MPTraits, template<typename> class Strategy>
typename SparkPRM<MPTraits, Strategy>::VID
SparkPRM<MPTraits, Strategy>::
ExpandTree(CfgType& _dir, vector<VID>& _rrt, vector<VID>& _important) {

  StatClass* stats = this->GetStatClass();
  GraphType* graph = this->GetRoadmap()->GetGraph();
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);
  NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
  VID recentVID = INVALID_VID;
  CDInfo cdInfo;
  CfgType nearest, newCfg;
  int weight = 0;
  vector<pair<VID, double> > kClosest;
  vector<CfgType> cfgs;

  // Find closest CFG in map
  stats->StartClock("RRT: ExpandTree: KClosest");
  nf->FindNeighbors(this->GetRoadmap(), _rrt.begin(), _rrt.end(), false, _dir,
      back_inserter(kClosest));
  stats->StopClock("RRT: ExpandTree: KClosest");
  nearest = graph->GetVertex(kClosest[0].first);

  // If connected too close to an important VID, don't expand
  if(m_checkImportant) {
    stats->StartClock("RRT: ExpandTree: Important");
    VID isImportant = INVALID_VID;
    for(typename vector<VID>::iterator it = _important.begin();
        it != _important.end(); it++) {
      CfgType _importantCfg = graph->GetVertex(*it);
      if(dm->Distance(nearest, _importantCfg) <= 1.01*m_delta) {
        isImportant = *it;
        break;
      }
    }
    stats->StopClock("RRT: ExpandTree: Important");
    if(isImportant != INVALID_VID)
      return recentVID;
  }

  // Expand the RRT
  typename MPProblemType::ExtenderPointer e = this->GetExtender(m_eLabel);
  LPOutput<MPTraits> lpOutput;
  stats->StartClock("RRT: ExpandTree: RRTExpand");
  if(!e->Extend(nearest, _dir, newCfg, lpOutput)) {
    stats->StopClock("RRT: ExpandTree: RRTExpand");
    return recentVID;
  }
  stats->StopClock("RRT: ExpandTree: RRTExpand");

  // If expansion was successful, add new vertex to roadmap
  if(dm->Distance(newCfg, nearest) >= m_minDist) {
    recentVID = graph->AddVertex(newCfg);
    _rrt.push_back(recentVID);

    pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand",
        weight), WeightType("RRTExpand", weight));
    graph->AddEdge(kClosest[0].first, recentVID, weights);
    graph->GetVertex(recentVID).SetStat("Parent", kClosest[0].first);
  }
  return recentVID;
}

#endif
