// A map evaluator that runs a query. Stops running when query returns a valid path.
// If no path is found, returns to sampling again.

#ifndef QUERY_H_
#define QUERY_H_

#include "MapEvaluatorMethod.h"
#include "LocalPlanners/LPOutput.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MedialAxisUtilities.h"
#include "LocalPlanners/MedialAxisLP.h"

template<class MPTraits>
class Query : public MapEvaluatorMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    Query(bool _deleteNodes=false, string _searchAlg="astar", string _pathFile="Basic.path", string _smoothFile="Basic.smooth.path",
    string _maSmoothFile="Basic.maSmooth.path", string _intermediateFile="", string lpLabel="", string _dmLabel="", string _malpLabel="",
    bool _smooth=false, bool _maSmooth=false);

    Query(string _queryFileName);
    Query(const CfgType& _start, const CfgType& _goal);
    Query(MPProblemType* _problem, XMLNodeReader& _node, bool _warn = true);
    virtual ~Query() { }

    void ParseXML(XMLNodeReader& _node);
    virtual void PrintOptions(ostream& _os);
    vector<CfgType>& GetQuery() { return m_query; }
    vector<CfgType>& GetPath() { return m_path; }
    void SetPathFile(string _filename) { m_pathFile = _filename; }

    // Reads a query and calls the other PerformQuery(), then calls Smooth() if desired
    virtual bool PerformQuery(RoadmapType* _rdmp);

    // Performs the real query work
    virtual bool PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp);

    // Smooths the query path by skipping intermediate nodes in the path
    virtual void Smooth();

    // Smooths the query path pushing it to the medial axis
    virtual void MedialAxisSmooth();

    // Checks if a path is valid
    virtual bool CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath);

    // Node enhancement step, does nothing in Query; used in LazyPRMQuery, for example
    virtual void NodeEnhance(RoadmapType* _rdmp) { }

    virtual void ReadQuery(string _filename);

    virtual bool operator()();

  protected:
    enum GraphSearchAlg {DIJKSTRAS, ASTAR};
    vector<CfgType> m_query;    // Holds the start and goal CfgTypes
    vector<CfgType> m_path;     // The path found
    string m_queryFile;         // Where to read in the query
    string m_pathFile;          // Where to write the initial unsmoothed path
    string m_smoothFile;        // Where to write the smoothed path
    string m_maSmoothFile;      // Where to write the medial-axis smoothed path
    string m_intermediateFile;  // Where to output the intermediate CfgTypes along the path if != ""
    string m_lpLabel;           // Local planner
    string m_dmLabel;           // Distance metric
    string m_malpLabel;         // Medial axis local planner
    bool m_deleteNodes;         // Delete any added nodes?
    bool m_smooth;              // Perform smoothing operation?
    bool m_maSmooth;            // Perform medial axis smoothing operation?
    GraphSearchAlg m_searchAlg; // Shortest-path graph search algorithm
    vector<string> m_nodeConnectionLabels;   // List of connection methods for query

  private:
    //initialize variable defaults
    void Initialize();
    void SetSearchAlgViaString(string _alg);
    void AddToPath(LPOutput<MPTraits>* _lpOutput, CfgType& _end);

    vector<VID> m_pathVIDs;     // Stores path nodes for easy reference during smoothing
    vector<VID> m_toBeDeleted;  // Nodes to be deleted if m_deleteNodes is enabled.
};

// Heuristic for A* graph search
template<class MPTraits>
struct Heuristic {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;

    Heuristic(const CfgType& _goal, double _posRes, double _oriRes) :
      m_goal(_goal), m_posRes(_posRes), m_oriRes(_oriRes) { }

    WeightType operator()(const CfgType& _c1) {
      int tick;
      CfgType incr;
      incr.FindIncrement(_c1, m_goal, &tick, m_posRes, m_oriRes);
      return WeightType("", tick/2);
    }

  private:
    CfgType m_goal;
    double m_posRes;
    double m_oriRes;
};

template<class MPTraits>
Query<MPTraits>::Query(bool _deleteNodes, string _searchAlg, string _pathFile, string _smoothFile, string _maSmoothFile,
    string _intermediateFile, string _lpLabel, string _dmLabel, string _malpLabel, bool _smooth, bool _maSmooth) :
  m_pathFile(_pathFile), m_smoothFile(_smoothFile), m_maSmoothFile(_maSmoothFile), m_intermediateFile(_intermediateFile),
  m_lpLabel(_lpLabel), m_dmLabel(_dmLabel), m_malpLabel(_malpLabel), m_deleteNodes(_deleteNodes), m_smooth(_smooth),
  m_maSmooth(_maSmooth){
    this->SetName("Query");
    SetSearchAlgViaString(_searchAlg);
}

// Reads in query from a file
template<class MPTraits>
Query<MPTraits>::Query(string _queryFileName) {
  Initialize();
  ReadQuery(_queryFileName);
}

// Uses start/goal to set up query
template<class MPTraits>
Query<MPTraits>::Query(const CfgType& _start, const CfgType& _goal) {
  Initialize();
  m_query.push_back(_start);
  m_query.push_back(_goal);
}

// Constructor with XML
template<class MPTraits>
Query<MPTraits>::Query(MPProblemType* _problem, XMLNodeReader& _node, bool _warn) :
  MapEvaluatorMethod<MPTraits>(_problem, _node) {
    Initialize();
    ParseXML(_node);
    if(_warn)
      _node.warnUnrequestedAttributes();
    ReadQuery(m_queryFile);
  }

template<class MPTraits>
void
Query<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_queryFile = _node.stringXMLParameter("queryFile", true, "", "Query filename");
  m_pathFile = _node.stringXMLParameter("pathFile", false, "", "Query output path filename");
  m_smooth = _node.boolXMLParameter("smooth", false, false, "Whether or not to smooth the path");
  m_maSmooth = _node.boolXMLParameter("maSmooth", false, false, "Whether or not to apply the medial axis smoothing to the path");
  m_smoothFile = _node.stringXMLParameter("smoothFile", false, "", "Smoothed path filename");
  m_maSmoothFile = _node.stringXMLParameter("maSmoothFile", false, "", "Medial axis smoothed path filename");
  m_lpLabel = _node.stringXMLParameter("lpMethod", true, "", "Local planner method");
  m_dmLabel = _node.stringXMLParameter("dmMethod", false, "", "Distance metric method");
  m_malpLabel = _node.stringXMLParameter("malpMethod", m_maSmooth, "", "Medial axis local planner label needed by MedialAxisSmooth");
  string searchAlg = _node.stringXMLParameter("graphSearchAlg", false, "dijkstras", "Graph search algorithm");
  m_intermediateFile = _node.stringXMLParameter("intermediateFiles", false, "", "Determines output location of intermediate nodes.");
  m_deleteNodes = _node.boolXMLParameter("deleteNodes", false, false, "Whether or not to delete start and goal from roadmap");

  for(XMLNodeReader::childiterator citr = _node.children_begin(); citr != _node.children_end(); ++citr) {
    if(citr->getName() == "NodeConnectionMethod") {
      m_nodeConnectionLabels.push_back(citr->stringXMLParameter("method", true, "", "Node connection method"));
      citr->warnUnrequestedAttributes();
    }
  }

  if(m_nodeConnectionLabels.empty()) {
    cerr << "\n\nWARNING:: in Query XML constructor:: no node connection methods specified. Default will be used." << endl;
  }

  // Ignore case for graph search algorithm
  transform(searchAlg.begin(), searchAlg.end(), searchAlg.begin(), ::tolower);
  SetSearchAlgViaString(searchAlg);
}

template<class MPTraits>
void
Query<MPTraits>::PrintOptions(ostream& _os) {
  _os << this->GetNameAndLabel() << "::";
  _os << "\n\tquery file = \"" << m_queryFile << "\"";
  _os << "\n\tpath file = \"" << m_pathFile << "\"";
  _os << "\n\tsmooth path file = \"" << m_smoothFile << "\"";
  _os << "\n\tmedial axis smooth path file = \"" << m_maSmoothFile << "\"";
  _os << "\n\tintermediate file = \"" << m_intermediateFile << "\"";
  _os << "\n\tdistance metric = " << m_dmLabel;
  _os << "\n\tlocal planner = " << m_lpLabel;
  _os << "\n\tmedial axis local planner = " << m_malpLabel;
  _os << "\n\tsearch alg = " << m_searchAlg;
  _os << "\n\tsmooth = " << m_smooth;
  _os << "\n\tmaSmooth = " << m_maSmooth;
  _os << "\n\tdeleteNodes = " << m_deleteNodes << endl;
}

// Runs the query
template<class MPTraits>
bool
Query<MPTraits>::operator()() {

  RoadmapType* rdmp = this->GetMPProblem()->GetRoadmap();

  // Perform query
  bool ans = PerformQuery(rdmp);

  // Delete added nodes (such as start and goal) if desired
  if(m_deleteNodes) {
    for(typename vector<VID>::iterator it = m_toBeDeleted.begin(); it != m_toBeDeleted.end(); it++)
      rdmp->GetGraph()->delete_vertex(*it);
    m_toBeDeleted.clear();
  }

  return ans;
}

// Reads the query and calls the other PerformQuery method
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(RoadmapType* _rdmp) {
  if(m_query.empty())
    cerr << "Error in Query::PerformQuery() because m_query is empty.\n"
      << "Sometimes caused by reading the wrong query file in the XML." << endl;

  //clear out path before calling query to ensure that only portions relevant to
  //the query appear in the overall output.
  m_path.clear();
  m_pathVIDs.clear();

  for(typename vector<CfgType>::iterator it = m_query.begin(); it+1 != m_query.end(); it++) {
    if(this->m_debug) {
      cout << "\n*Q* query is ...     ";
      cout << *it;
      cout << "\n*Q*                  ";
      cout << *(it+1);
      cout << "\n*Q* working  ..." << endl;
    }

    if(!PerformQuery(*it, *(it+1), _rdmp)) {
      if(this->m_debug)
        cout << endl << "*Q* In Query::PerformQuery(): didn't connect";
      return false;
    }
  }

  if(m_pathFile == "") {
    cerr << "Warning: no path file specified. Outputting path to \"Basic.path\"." << endl;
    WritePath("Basic.path", m_path);
  }
  else{
    WritePath(m_pathFile, m_path);
  }
  if(m_smooth || m_maSmooth)
    Smooth();
  if(m_maSmooth)
    MedialAxisSmooth();
  return true;
}

// Performs the query
template<class MPTraits>
bool
Query<MPTraits>::PerformQuery(const CfgType& _start, const CfgType& _goal, RoadmapType* _rdmp) {

  if(this->m_debug)
    cout << "*Q* Begin query" << endl;
  VDComment("Begin Query");

  StatClass* stats = this->GetMPProblem()->GetStatClass();
  static int graphSearchCount = 0;
  LPOutput<MPTraits> sci, gci; // Connection info for start, goal nodes
  vector<pair<size_t, VID> > ccs;
  stapl::sequential::vector_property_map<GraphType, size_t> cmap;
  if(this->m_recordKeep)
    stats->IncGOStat("CC Operations");
  get_cc_stats(*(_rdmp->GetGraph()), cmap, ccs);
  bool connected = false;

  if(this->m_debug)
    cout << "*Q* There are " << ccs.size() << " CCs." << endl;

  // Process node connection labels
  vector<ConnectorPointer> connectionMethods;
  if(m_nodeConnectionLabels.empty())
    m_nodeConnectionLabels.push_back("");

  for(vector<string>::iterator it = m_nodeConnectionLabels.begin(); it != m_nodeConnectionLabels.end(); it++)
    connectionMethods.push_back(this->GetMPProblem()->GetConnector(*it));

  // Add start and goal to roadmap (if not already there)
  VID sVID, gVID;
  if(_rdmp->GetGraph()->IsVertex(_start))
    sVID = _rdmp->GetGraph()->GetVID(_start);
  else {
    sVID = _rdmp->GetGraph()->AddVertex(_start);
    m_toBeDeleted.push_back(sVID);
  }
  if(_rdmp->GetGraph()->IsVertex(_goal))
    gVID = _rdmp->GetGraph()->GetVID(_goal);
  else {
    gVID = _rdmp->GetGraph()->AddVertex(_goal);
    m_toBeDeleted.push_back(gVID);
  }

  // Loop through connected components
  typename vector<pair<size_t, VID> >::const_iterator ccIt, ccsBegin = ccs.begin();
  for(ccIt = ccsBegin; ccIt != ccs.end(); ccIt++) {
    // Store cc VIDs if needed, shortest path for smoothing later if needed
    vector<VID> cc, shortestPath;

    // Try to connect start to cc
    cmap.reset();
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Start already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      cmap.reset();
      if(this->m_recordKeep)
        stats->IncGOStat("CC Operations");
      stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      vector<VID> verticesList(1, sVID);
      if(this->m_debug)
        cout << "*Q* Connecting start to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(typename vector<ConnectorPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, *stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Try to connect goal to cc
    cmap.reset();
    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");
    if(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, gVID, ccIt->second)) {
      if(this->m_debug)
        cout << "*Q* Goal already connected to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;
    }
    else {
      if(cc.empty()) {
        cmap.reset();
        if(this->m_recordKeep)
          stats->IncGOStat("CC Operations");
        stapl::sequential::get_cc(*(_rdmp->GetGraph()), cmap, ccIt->second, cc);
      }
      vector<VID> verticesList(1, gVID);
      if(this->m_debug)
        cout << "*Q* Connecting goal to ccIt[" << distance(ccsBegin, ccIt)+1 << "]" << endl;

      for(typename vector<ConnectorPointer>::iterator
          itr = connectionMethods.begin(); itr != connectionMethods.end(); itr++) {
        cmap.reset();
        (*itr)->Connect(_rdmp, *stats, cmap, verticesList.begin(), verticesList.end(), cc.begin(), cc.end());
      }
    }

    // Check if start and goal are connected to the same CC
    cmap.reset();
    while(stapl::sequential::is_same_cc(*(_rdmp->GetGraph()), cmap, sVID, gVID)) {
      if(this->m_recordKeep)
        stats->IncGOStat("CC Operations");
      //get DSSP path
      shortestPath.clear();
      cmap.reset();
      //TO DO:: fix compilation issue in parallel
#ifndef _PARALLEL
      // Run a graph search
      graphSearchCount++;
      if(this->m_recordKeep) {
        stats->IncGOStat("Graph Search");
        stats->StartClock("Query Graph Search");
      }
      switch(m_searchAlg) {
        case DIJKSTRAS:
          find_path_dijkstra(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, WeightType::MaxWeight());
          break;
        case ASTAR:
          Heuristic<MPTraits> heuristic(_goal, this->GetMPProblem()->GetEnvironment()->GetOrientationRes(),
              this->GetMPProblem()->GetEnvironment()->GetPositionRes());
          astar(*(_rdmp->GetGraph()), sVID, gVID, shortestPath, heuristic);
          break;
      }
      if(this->m_recordKeep)
        stats->StopClock("Query Graph Search");
#endif
      if(this->m_debug)
        cout << "*Q* Start(" << shortestPath[1] << ") and Goal(" << shortestPath[shortestPath.size()-2]
          << ") seem connected to same ccIt[" << distance(ccsBegin, ccIt)+1  << "]!" << endl;

      // Attempt to recreate path
      vector<CfgType> recreatedPath;
      if(CanRecreatePath(_rdmp, shortestPath, recreatedPath)) {
        connected = true;
        m_path.insert(m_path.end(), recreatedPath.begin(), recreatedPath.end());
        if(m_smooth || m_maSmooth)
          m_pathVIDs.insert(m_pathVIDs.end(),shortestPath.begin(),shortestPath.end());
        break;
      }
      else if(this->m_debug)
        cout << endl << "*Q* Failed to recreate path\n";
    }

    if(this->m_recordKeep)
      stats->IncGOStat("CC Operations");

    if(connected) {
      if(m_intermediateFile != "") {
        // Print out all start, all graph nodes, and goal; no "ticks" from local planners
        vector<CfgType> mapCfgs;
        for(typename vector<VID>::iterator it = shortestPath.begin(); it != shortestPath.end(); it++)
          mapCfgs.push_back(_rdmp->GetGraph()->GetVertex(*it));
        WritePath(m_intermediateFile, mapCfgs);
      }
      break;
    }
  }

  if(!connected)
    NodeEnhance(_rdmp);

  if(this->m_debug)
    cout << "*Q* Ending query with: " << _rdmp->GetGraph()->get_num_vertices() << " vertices, "
      << _rdmp->GetGraph()->get_num_edges()/2 << " edges."
      << " graphSearchCount = " << graphSearchCount << ". Returning connected = " << connected << endl;
  VDComment("End Query");
  return connected;
}

/*TODO
 * When a query has many goals, the Smoothing operations do not take that into account.
 * i.e. A smoothed path is not likely to include all the goals in the final path
 * */

// Shortens the path by skipping nodes with a greedy approach.
// This function is also used by MedialAxisSmooth. However, it only prints the smoothed path if m_smooth is true.
template<class MPTraits>
void
Query<MPTraits>::Smooth() {
  if(this->m_debug) cout << "\n*S* Executing Query::Smooth()" << endl;
  if(!m_pathVIDs.empty()){
    bool reconstruct, skip;
    LocalPlannerPointer lp = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
    Environment* env = this->GetMPProblem()->GetEnvironment();
    StatClass* stats = this->GetMPProblem()->GetStatClass();
    DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
    GraphType* graph = this->GetMPProblem()->GetRoadmap()->GetGraph();
    LPOutput<MPTraits> tmpOutput;
    double posRes = env->GetPositionRes();
    double oriRes = env->GetOrientationRes();

    if(this->m_recordKeep) stats->StartClock("Path Smoother");
    //This variable will store how many nodes were skipped
    size_t skips = 0;

    vector<VID> oldPathVIDs = m_pathVIDs;
    m_pathVIDs.clear();
    m_path.clear();

    //Save the first node in the path. (i.e. it can never be skipped)
    m_pathVIDs.push_back(oldPathVIDs[0]);
    if(m_smooth){
      m_path.push_back(graph->GetVertex(oldPathVIDs[0]));
    }

    size_t n = oldPathVIDs.size();
    size_t i = 0;   //i is the index of the start VID (configuration)
    size_t j = n-1; //j is the index of the goal VID (configuration)
    while(i<j){
      if((i+1)==j){//If the vertices are adjacent, there are no nodes to skip
        //Only reconstruct and save path if smoothing output is wanted.
        if(m_smooth){
          reconstruct = lp->IsConnected(env, *stats, dm, graph->GetVertex(oldPathVIDs[i]), graph->GetVertex(oldPathVIDs[j]), &tmpOutput, posRes, oriRes, true, true, true);
          AddToPath(&tmpOutput, graph->GetVertex(oldPathVIDs[j]));
          if(!reconstruct){ //If could not reconstruct, abort the output, but continue skipping nodes
            cerr << "*S* Error. Could not reconstruct path in Query::Smooth()" << endl;
            m_smooth = false;
            if(m_path.empty()){
              cerr << "*S*\tm_path is empty. Cannot write anything" << endl;
            }
            else{
              cerr << "*S*\tWill output failed path to \"error.smooth.path\"" << endl;
              WritePath("error.smooth.path", m_path);
              m_path.clear();
            }
          }
        }
        m_pathVIDs.push_back(oldPathVIDs[j]);
        i = j;
        j = n-1;
      }
      else{
        //Try to make a connection by skipping nodes betwen i and j
        skip = lp->IsConnected(env, *stats, dm, graph->GetVertex(oldPathVIDs[i]), graph->GetVertex(oldPathVIDs[j]), &tmpOutput, posRes, oriRes, true, true);
        if(skip){
          if(m_smooth){
            AddToPath(&tmpOutput, graph->GetVertex(oldPathVIDs[j]));
          }
          m_pathVIDs.push_back(oldPathVIDs[j]);
          skips += (j-i-1);
          i = j;
          j = n-1;
        }
        else{
          //Could not skip. Try the previous node.
          --j;
        }
      }
    }

    if(this->m_debug){
      if(skips==0){
        cout << "*S* Could not skip any nodes" << endl;
      }
      else{
        cout << "*S* Smoothing operation skipped " << skips;
        if(skips==1){
          cout << " node" << endl;
        }
        else{
          cout << " nodes" << endl;
        }
      }
    }

    //Output the path if it is wanted
    if(m_smooth){
      if(m_smoothFile==""){
        cerr << "*S* Warning: no smooth path file specified. Outputting smoothed path to \"Basic.smooth.path\"." << endl;
        WritePath("Basic.smooth.path", m_path);
      }
      else{
        if(this->m_debug) cout << "*S* Writing smooth path into \"" << m_smoothFile << "\"" << endl;
        WritePath(m_smoothFile, m_path);
      }
    }
    if(this->m_recordKeep) stats->StopClock("Path Smoother");
  }
  else{
    cerr << "*S* Error. m_pathVIDs in " << this->GetNameAndLabel() << " is empty. Aborting smoothing operation(s)." << endl;
    m_maSmooth = false;
  }
}

template<class MPTraits>
void
Query<MPTraits>::MedialAxisSmooth(){
  //This function assumes that m_pathVIDs is not empty because the previous call to
  //Smooth() should prevent the call of this function whenever it is empty
  if(this->m_debug) cout << "\n*M* Executing Query::MedialAxisSmooth()" << endl;

  MedialAxisLP<MPTraits>* maLP = dynamic_cast<MedialAxisLP<MPTraits>*>(
      this->GetMPProblem()->GetLocalPlanner(m_malpLabel).get());
  if(maLP){
    bool result = true;
    StatClass* stats = this->GetMPProblem()->GetStatClass();
    GraphType* graph = this->GetMPProblem()->GetRoadmap()->GetGraph();
    shared_ptr<Boundary> bBox = this->GetMPProblem()->GetEnvironment()->GetBoundary();
    MedialAxisUtility<MPTraits>& mau = maLP->GetMedialAxisUtility();

    if(this->m_recordKeep) stats->StartClock("Medial Axis Path Smoother");
    size_t n = m_pathVIDs.size();

    //Copy all the nodes from m_pathVIDs to avoid modifying them
    vector<CfgType> pushedNodes;
    size_t i;
    for(i=0;i<n;++i){
      pushedNodes.push_back(graph->GetVertex(m_pathVIDs[i]));
    }

    //Copying the start and goal nodes
    CfgType& start = pushedNodes[0];
    CfgType& goal = pushedNodes[n-1];

    //Push all the nodes of the path
    i = 0;
    while(result && i<n){
      result = mau.PushToMedialAxis(pushedNodes[i], bBox);
      //Print a debug statement if succeeded.
      if(this->m_debug && result)
        cout << "*M* Node " << i << " successfully pushed" << endl;
      ++i;
    }
    if(result){//If all the nodes were pushed correctly
      //Create the variables used to connect the nodes
      Environment* env = this->GetMPProblem()->GetEnvironment();
      DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
      LocalPlannerPointer envLP = this->GetMPProblem()->GetLocalPlanner(m_lpLabel);
      LPOutput<MPTraits> tmpOutput;
      double posRes = env->GetPositionRes();
      double oriRes = env->GetOrientationRes();

      //Make room for the path
      m_path.clear();

      //Connect the start configuration and its pushed version
      result = envLP->IsConnected(env, *stats, dm, start, pushedNodes[0], &tmpOutput, posRes, oriRes, true, true, true);
      if(result){
        m_path.push_back(start);
        AddToPath(&tmpOutput, pushedNodes[0]);

        //Connect the nodes that are already in the medial axis
        i = 1;
        while(result && i<n){
          result = maLP->LocalPlannerMethod<MPTraits>::IsConnected(env, *stats, dm, pushedNodes[i-1], pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
          if(result){
            AddToPath(&tmpOutput, pushedNodes[i]);
          }
          else{ //Failure control measures (FCM)
            if(this->m_debug) cout << "*M*\t" << maLP->GetNameAndLabel() << " failed to connect the pair of nodes (" << (i-1) << ", " << i << ")" << endl
              << "*M*\tAttempting Failure Control Measures (FCM):" << endl;
            //First FCM: Try with the other local planner
            result = envLP->IsConnected(env, *stats, dm, pushedNodes[i-1], pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
            if(result){
              if(this->m_debug) cout << "*M*\t\tFCM1: " << envLP->GetNameAndLabel() << " succeeded" << endl;
              AddToPath(&tmpOutput, pushedNodes[i]);
            }
            else{
              if(this->m_debug) cout << "*M*\t\tFCM1: " << envLP->GetNameAndLabel() << " also failed" << endl;
              //Second FCM: Connect the nodes in the medial axis to the original path and back
              //This FCM should not fail, unless envLP is very badly chosen
              //Copying the old configurations
              CfgType oldCfg1 = graph->GetVertex(m_pathVIDs[i-1]);
              CfgType oldCfg2 = graph->GetVertex(m_pathVIDs[i]);
              result = envLP->IsConnected(env, *stats, dm, pushedNodes[i-1], oldCfg1, &tmpOutput, posRes, oriRes, true, true, true);
              if(result){
                if(this->m_debug) cout << "*M*\t\tFCM2: First connection established" << endl;
                AddToPath(&tmpOutput, oldCfg1);
                result = envLP->IsConnected(env, *stats, dm, oldCfg1, oldCfg2, &tmpOutput, posRes, oriRes, true, true, true);
              }
              if(result){
                if(this->m_debug) cout << "*M*\t\tFCM2: Second connection established" << endl;
                AddToPath(&tmpOutput, oldCfg2);
                result = envLP->IsConnected(env, *stats, dm, oldCfg2, pushedNodes[i], &tmpOutput, posRes, oriRes, true, true, true);
              }
              if(result){
                if(this->m_debug) cout << "*M*\t\tFCM2: Succeeded" << endl;
                AddToPath(&tmpOutput, pushedNodes[i]);
              }
              else{
                if(this->m_debug) cout << "*M*\t\tFCM2: Failed" << endl;
              }
            }
          }
          ++i;
        }
        if(result){//If all the intermediate connections succeeded
          //Connect the goal configuration and its pushed version
          result = envLP->IsConnected(env, *stats, dm, pushedNodes[n-1], goal, &tmpOutput, posRes, oriRes, true, true, true);
          if(result){
            //Add to path
            AddToPath(&tmpOutput, goal);
            /*TODO
             * Remove Branches Algorithm (from m_path)
             * */

            //Write the path
            if(m_maSmoothFile==""){
              cerr << "*M* Warning: maSmoothFile was not specified. Outputting smoothed path to "
                << "\"Basic.maSmooth.path\" instead." << endl;
              WritePath("Basic.maSmooth.path", m_path);
            }
            else{
              if(this->m_debug) cout << "*M* Writing medial axis smoothed path into \""
                << m_maSmoothFile << "\"" << endl;
              WritePath(m_maSmoothFile, m_path);
            }
          }
        }
        else{
          //If this happens, another lpLabel should be used.
          //vcLabel in malpLabel could be causing trouble too
          if(this->m_debug) cout << "*M* \tSecond failure control measure also failed" << endl;
        }
      }
      else{
        //If this happens, another lpLabel should be used.
        //A medial axis local planner is not a good choice when the start and goal are not on the medial axis.
        if(this->m_debug) cout << "*M* Local Planner " << envLP->GetNameAndLabel() << " could not connect the start to its pushed version" << endl;
      }
    }
    else{
      if(this->m_debug) cout << "*M* Could not push the configuration in m_pathVIDs[" << (i-1) << "]" << endl;
    }
    if(this->m_recordKeep) stats->StopClock("Medial Axis Path Smoother");
  }
  else{
    cerr << "*M* m_malpLabel = \"" << m_malpLabel << "\" in " << this->GetNameAndLabel()
      << " needs to point to a MedialAxisLP. Will not execute MedialAxisSmooth()" << endl;
  }
}

//Auxiliary function created to avoid checking emptiness everytime
//Adds the path that is stored in lpOutput to m_path as well as the _end configuration
template<class MPTraits>
void
Query<MPTraits>::AddToPath(LPOutput<MPTraits>* _lpOutput, CfgType& _end){
  if(!_lpOutput->path.empty()){
    m_path.insert(m_path.end(), _lpOutput->path.begin(), _lpOutput->path.end());
  }
  m_path.push_back(_end);
}

// Checks validity of path, recreates path if possible
template<class MPTraits>
bool
Query<MPTraits>::CanRecreatePath(RoadmapType* _rdmp, vector<VID>& _attemptedPath, vector<CfgType>& _recreatedPath) {
  StatClass* stats = this->GetMPProblem()->GetStatClass();
  _recreatedPath.push_back(_rdmp->GetGraph()->GetVertex(*(_attemptedPath.begin())));
  for(typename vector<VID>::iterator it = _attemptedPath.begin(); it+1 != _attemptedPath.end(); it++) {
    LPOutput<MPTraits> ci;
    typename GraphType::vertex_iterator vi;
    typename GraphType::adj_edge_iterator ei;
    typename GraphType::edge_descriptor ed(*it, *(it+1));
    _rdmp->GetGraph()->find_edge(ed, vi, ei);
    CfgType col;

    if(this->GetMPProblem()->GetLocalPlanner(m_lpLabel)->IsConnected(
          this->GetMPProblem()->GetEnvironment(), *stats, this->GetMPProblem()->GetDistanceMetric(m_dmLabel),
          _rdmp->GetGraph()->GetVertex(*it), _rdmp->GetGraph()->GetVertex(*(it+1)),
          col, &ci, this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
          this->GetMPProblem()->GetEnvironment()->GetOrientationRes(), true, true, true)) {
      _recreatedPath.insert(_recreatedPath.end(), ci.path.begin(), ci.path.end());
      _recreatedPath.push_back(_rdmp->GetGraph()->GetVertex(*(it+1)));
    }
    else {
      cerr << "Error::When querying, invalid edge of graph was found between vid pair ("
        << *it << ", " << *(it+1) << ")" << " outputting error path to \"error.path\" and exiting." << endl;
      _recreatedPath.insert(_recreatedPath.end(), ci.path.begin(), ci.path.end());
      _recreatedPath.push_back(col);
      WritePath("error.path", _recreatedPath);
      exit(1);
    }
  }
  return true;
}

// Reads query CfgTypes from file
template<class MPTraits>
void
Query<MPTraits>::ReadQuery(string _filename) {
  CfgType tempCfg;
  ifstream in(_filename.c_str());
  if(!in) {
    cout << endl << "In ReadQuery: can't open infile: " << _filename << endl;
    return;
  }
  in >> tempCfg;
  while(in) {
    m_query.push_back(tempCfg);
    in >> tempCfg;
  }
  in.close();
}

//initialize variable defaults
template<class MPTraits>
void
Query<MPTraits>::Initialize() {
  this->SetName("Query");
  m_searchAlg = ASTAR;
  m_pathFile = "Basic.path";
  m_smoothFile = "Basic.smooth.path";
  m_maSmoothFile = "Basic.maSmooth.path";
  m_intermediateFile = "";
  m_lpLabel = "";
  m_dmLabel = "";
  m_malpLabel = "";
  m_deleteNodes = false;
  m_smooth = false;
  m_maSmooth = false;
}

template<class MPTraits>
void
Query<MPTraits>::SetSearchAlgViaString(string _alg){
  if(_alg == "dijkstras")
    m_searchAlg = DIJKSTRAS;
  else if(_alg == "astar")
    m_searchAlg = ASTAR;
  else {
    cout << "Error: invalid graphSearchAlg; valid choices are: \"dijkstras\", \"astar\". Exiting." << endl;
    exit(1);
  }
}

#endif
