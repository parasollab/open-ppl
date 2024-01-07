#include "PathStrategy.h"

#include "MPLibrary/MPLibrary.h"

#include <iterator>
#include <set>
#include "Utilities/Hash.h"

/*------------------------------- Construction -------------------------------*/

PathStrategy::PathStrategy() : MPStrategyMethod() {
  this->SetName("PathStrategy");
}

PathStrategy::PathStrategy(XMLNode& _node) : MPStrategyMethod(_node) {
  this->SetName("PathStrategy");
  m_vcLabel = _node.Read("vcLabel", false, m_vcLabel, "Validity Checker");
  m_dmLabel = _node.Read("dmLabel", false, m_dmLabel, "Distance Metric");
  m_lpLabel = _node.Read("lpLabel", false, m_lpLabel, "Local Planner");
  m_ncLabel = _node.Read("connectionLabel", false, "Closest", "Connector");
  m_radius = _node.Read("radius", false, m_radius, 0.0, 100.0, "Search Radius");
  m_UserPathsFilename =
      _node.Read("pathFile", false, m_UserPathsFilename, "User Paths File");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void PathStrategy::Print(ostream& _os) const {
  _os << this->GetNameAndLabel() << "\n\tValidity Checker: " << m_vcLabel
      << "\n\tDistance Metric:  " << m_dmLabel
      << "\n\tLocal Planner:    " << m_lpLabel
      << "\n\tConnector:        " << m_ncLabel
      << "\n\tRadius:           " << m_radius << endl;
}

/*------------------------- MPStrategyMethod Overrides -----------------------*/

void PathStrategy::Initialize() {
  // This function is a no-op if no paths exist.
  getUserPaths();
  if (m_userPaths.empty())
    return;

  cout << "Initializing Path Strategy." << endl;

  this->GetStatClass()->StartClock("PathStrategyMP");
}

void PathStrategy::Run() {
  // This function is a no-op if no paths exist.
  if (m_userPaths.empty())
    return;

  cout << "Running Path Strategy." << endl;
  // iterate through all paths to build and connect their c-space analogs
  typedef typename vector<vector<Point3d>>::iterator PIT;
  for (PIT path = m_userPaths.begin(); path != m_userPaths.end(); ++path) {
    // add path to roadmap
    vector<VID> vids;
    AddToRoadmap(*path, vids);

    // store head and tail
    m_endPoints.push_back(vids.front());
    m_endPoints.push_back(vids.back());

    // connect cfgs with path connector
    ConnectPath(vids);
  }

  // try to connect all endpoints
  auto connector = this->GetMPLibrary()->GetConnector(m_ncLabel);
  auto map = this->GetRoadmap();

  const VertexSet targetSet(m_endPoints.begin(), m_endPoints.end());
  connector->Connect(map, m_endPoints.begin(), m_endPoints.end());

  // try to connect remaining connected components
  ConnectCCs();
}

void PathStrategy::Finalize() {
  if (m_userPaths.empty()) {
    cout << "No user-path exists!" << endl;
    return;
  }

  cout << "Finalizing Path Strategy." << endl;

  auto stats = this->GetStatClass();
  stats->StopClock("PathStrategyMP");

  // Print clocks to terminal.
  stats->PrintClock("PathStrategyMP", cout);

  string basename = this->GetBaseFilename();

  // Output stats.
  ofstream ostats(basename + ".stat");
  ostats << "Sampling and Connection Stats:" << endl;
  auto map = this->GetRoadmap();
  stats->PrintAllStats(ostats, map);
  stats->PrintClock("PathStrategyMP", ostats);

  // Output roadmap.
  Environment* env = this->GetEnvironment();
  map->Write(basename + ".map", env);
}

/*--------------------------------- Helpers ----------------------------------*/

void PathStrategy::getUserPaths() {
  ifstream ifs(m_UserPathsFilename);

  // read num paths
  string temp;
  size_t number;
  ifs >> temp >> temp >> number;
  m_userPaths.clear();
  m_userPaths.resize(number);
  for (int i = 0; i < int(number); ++i) {
    // Reading input type
    string temp;
    ifs >> temp >> temp;

    // read validity
    ifs >> temp >> temp;
    // read points
    size_t numpoints;
    ifs >> temp >> numpoints;
    vector<Point3d> path(numpoints);
    m_userPaths[i] = path;
    for (auto& point : m_userPaths[i])
      ifs >> point;
  }
}

bool PathStrategy::ValidateCfg(Cfg& _cfg) {
  // check the configuration is in bounds
  auto env = this->GetEnvironment();
  if (!_cfg.InBounds(env))
    return false;

  // check the validity of configuration
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  return vc->IsValid(_cfg, this->GetNameAndLabel());
}

void PathStrategy::AddToRoadmap(vector<Point3d>& _points, vector<VID>& _vids) {
  _vids.clear();
  auto g = this->GetRoadmap();
  Cfg blank(this->GetTask()->GetRobot());
  Cfg lastValidNode = Cfg(this->GetTask()->GetRobot());

  for (uint i = 0; i < _points.size() - 1; ++i) {
    Cfg cfg(_points[i], this->GetTask()->GetRobot());

    // if the configuration is valid add it to the roadmap
    if (ValidateCfg(cfg)) {
      VID addedNode = g->AddVertex(cfg);
      _vids.push_back(addedNode);
      lastValidNode = cfg;
    }  // if the configuration is invalid sample a near configuration
    else {
      Cfg nextCfg(_points[i + 1], this->GetTask()->GetRobot());
      cfg = RandomNearNode(cfg, nextCfg, lastValidNode);

      if (cfg != blank) {
        VID addedNode = g->AddVertex(cfg);
        _vids.push_back(addedNode);
        lastValidNode = cfg;
      }
    }
  }
}

void PathStrategy::ConnectPath(const vector<VID>& _vids) {
  LPOutput lpOutput;
  auto g = this->GetRoadmap();

  // iterate through the path configurations and try to connect them
  for (typename vector<VID>::const_iterator vit = _vids.begin();
       vit != _vids.end() - 1; ++vit) {
    const VID& v1 = *vit;
    const VID v2 = *(vit + 1);
    const Cfg& c1 = g->GetVertex(v1);
    const Cfg& c2 = g->GetVertex(v2);

    // if cfgs are connectable, add edge to map
    auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
    auto env = this->GetEnvironment();
    if (lp->IsConnected(c1, c2, &lpOutput, env->GetPositionRes(),
                        env->GetOrientationRes()))
      g->AddEdge(v1, v2, lpOutput.m_edge);

    // if connection fails, generate midpoint, validate it, and recurse
    else {
      Cfg cI = (c1 + c2) / 2.;

      // if cI can be validated, add it to the path and recurse on (c1, cI, c2)
      if (ValidateCfg(cI)) {
        vector<VID> recurseList(3);
        recurseList[0] = v1;
        recurseList[1] = g->AddVertex(cI);
        recurseList[2] = v2;
        ConnectPath(recurseList);
      }
    }
  }
}

Cfg PathStrategy::RandomNearNode(Cfg& _invalidNode,
                                 Cfg& _nextNode,
                                 Cfg& _lastValidNode) {
  Cfg newCfg(this->GetTask()->GetRobot());
  auto dm = this->GetMPLibrary()->GetDistanceMetric(m_dmLabel);
  auto env = this->GetEnvironment();
  auto vc = this->GetMPLibrary()->GetValidityChecker(m_vcLabel);
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  LPOutput lpOutput;

  bool valid = ValidateCfg(_nextNode);
  for (int i = 0; i < 100; ++i) {
    // generate new random near node
    newCfg.GetRandomRay(m_radius, dm);
    newCfg *= DRand();
    newCfg += _invalidNode;

    // check its validity and if it can be connected to the previous node
    if (newCfg.InBounds(env) && vc->IsValid(newCfg, this->GetNameAndLabel()) &&
        lp->IsConnected(_lastValidNode, newCfg, &lpOutput,
                        env->GetPositionRes(), env->GetOrientationRes())) {
      // if the next node is valid check that the new configuration can be
      // connected to it
      if (valid) {
        if (lp->IsConnected(_nextNode, newCfg, &lpOutput, env->GetPositionRes(),
                            env->GetOrientationRes())) {
          return newCfg;
        }
      } else
        return newCfg;
    }
  }
  // if not found return the blank configuration
  Cfg blank(this->GetTask()->GetRobot());
  return blank;
}

void PathStrategy::ConnectCCs() {
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();

  VertexSet representatives;

  bool successConnection = true;
  // while two different CCs are successfully connected keep trying to connect
  // until the remaining CCs cannot be connected
  while (successConnection) {
    successConnection = false;
    representatives = ccTracker->GetRepresentatives();
    for (auto it1 = representatives.begin();
         it1 != representatives.end() && !successConnection; it1++) {
      for (auto it2 = next(it1, 1);
           it2 != representatives.end() && !successConnection; it2++) {
        successConnection = ConnectCC(*it1, *it2);
      }
    }
  }
}

bool PathStrategy::ConnectCC(VID _vid1, VID _vid2) {
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  auto env = this->GetEnvironment();
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);
  auto nf = this->GetMPLibrary()->GetNeighborhoodFinder(m_nfLabel);
  LPOutput lpOutput;

  const VertexSet* const cc1 = ccTracker->GetCC(_vid1);
  const VertexSet* const cc2 = ccTracker->GetCC(_vid2);

  for (auto it1 = (*cc1).begin(); it1 != (*cc1).end(); it1++) {
    auto cfg1 = g->GetVertex(*it1);
    vector<Neighbor> neighbors;
    nf->FindNeighbors(g, cfg1, *cc2, std::back_inserter(neighbors));
    // get the nearest neighbors to cfg1 belonging to the second CC
    for (auto neigh : neighbors) {
      auto cfg2 = g->GetVertex(neigh.target);
      if (lp->IsConnected(cfg1, cfg2, &lpOutput, env->GetPositionRes(),
                          env->GetOrientationRes())) {
        g->AddEdge(*it1, neigh.target, lpOutput.m_edge);
        return true;
      }
    }
  }
  return false;
}
/*----------------------------------------------------------------------------*/
