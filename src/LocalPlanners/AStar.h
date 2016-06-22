#ifndef ASTAR_H_
#define ASTAR_H_

#include "LocalPlannerMethod.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MedialAxisUtilities.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief A* like local planning (not true A*) which steps towards optimal
///        neighbors when it cannot progress to the goal.
/// @tparam MPTraits Motion planning universe
///
/// Abstract class for A* like local planning. This steps towards the goal until
/// it cannot, then it chooses an optimal neighbor. The algorithm is as follows:
///
/// while(goal not reached or tries < maxTries){
///   find neighbors;
///   step towards optimal neighbors;
/// }
/// if(goal reached) return connected;
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class AStar : public LocalPlannerMethod<MPTraits> {
  public:
    /// @{
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    /// @}

    AStar(const string& _vcLabel = "", size_t _maxTries = 0,
        size_t _numNeighbors = 0, size_t _histLength = 5, bool _saveIntermediates = false);

    AStar(MPProblemType* _problem, XMLNode& _node);

    virtual ~AStar();

    virtual void Print(ostream& _os) const;

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    virtual vector<CfgType> ReconstructPath(
        const CfgType& _c1, const CfgType& _c2, const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes) {
      vector<CfgType> tmp = _intermediates;
      return tmp;
    }

  protected:

    virtual bool IsConnectedOneWay(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false);

    virtual size_t ChooseOptimalNeighbor(CfgType& _col,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) = 0;

    virtual vector<CfgType> FindNeighbors(
        const CfgType& _current, const CfgType& _goal, const CfgType& _increment);

    string m_vcLabel;
    size_t m_maxTries;     // How many time will be tried to connect to goal
    size_t m_numNeighbors; // How many neighbors will be seached abound current Cfg
    size_t m_histLength;   // how many nodes should I keep track of for cycles
};

template <class MPTraits>
AStar<MPTraits>::AStar(const string& _vcLabel,
    size_t _maxTries, size_t _numNeighbors, size_t _histLength, bool _saveIntermediates) :
    LocalPlannerMethod<MPTraits>(_saveIntermediates),
    m_vcLabel(_vcLabel), m_maxTries(_maxTries),
    m_numNeighbors(_numNeighbors), m_histLength(_histLength) {
  this->SetName("AStar");
}

template <class MPTraits>
AStar<MPTraits>::AStar(MPProblemType* _problem, XMLNode& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node) {
  this->SetName("AStar");
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Label");
  m_maxTries = _node.Read("maxTries", true, 0, 0, MAX_INT, "n tries");
  m_numNeighbors = _node.Read("numNeighbors", true, 0, 0, MAX_INT, "n neighbors");
  m_histLength = _node.Read("histLength", false, 5, 0, MAX_INT,
      "history length for detecting cycles");
}

template <class MPTraits>
AStar<MPTraits>::~AStar() {}

template <class MPTraits>
void
AStar<MPTraits>::Print(ostream& _os) const {
  LocalPlannerMethod<MPTraits>::Print(_os);
  _os << "\tvc label : " << m_vcLabel
      << "\n\tmax tries = " << m_maxTries
      << "\n\tnum neighbors = " << m_numNeighbors
      << "\n\thist length = " << m_histLength
      << endl;
}

template <class MPTraits>
bool
AStar<MPTraits>::IsConnected(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
  //clear _lpOutput
  _lpOutput->Clear();
  bool connected = false;

  connected = IsConnectedOneWay(_c1, _c2,_col, _lpOutput,
      _positionRes, _orientationRes, _checkCollision, _savePath);

  if(!connected) { //try the other way
    connected = IsConnectedOneWay(_c2, _c1,_col, _lpOutput,
        _positionRes, _orientationRes, _checkCollision, _savePath);
    if (_savePath)
      reverse(_lpOutput->m_path.begin(), _lpOutput->m_path.end());
  }

  if(connected) {
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }

  return connected;
}

template <class MPTraits>
bool
AStar<MPTraits>::IsConnectedOneWay(
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath) {
    StatClass* stats = this->GetMPProblem()->GetStatClass();

  if(this->m_debug) {
    VDClearAll();
    VDAddTempCfg(_c1, false);
    VDAddTempCfg(_c2, false);
  }

  stats->IncLPAttempts(this->GetNameAndLabel());

  CfgType p = _c1;
  CfgType incr;
  vector<CfgType> neighbors;
  int nTicks;
  bool connected = true;
  size_t tries = 0;
  size_t iter = 0;
  deque<CfgType> hist; //hist for detecting cycles

  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);

  do {
    if(this->m_debug)
      VDAddTempCfg(p, true);

    //update cycle history
    hist.push_front(p);
    if(hist.size() > m_histLength)
      hist.pop_back();

    //find neighbors
    neighbors = FindNeighbors(p, _c2, incr);

    //neighbors all in collision
    if(neighbors.size() == 0) {
      connected = false;
      if(this->m_debug)
        cout << this->GetNameAndLabel() << ":: Found 0 Neighbors" << endl;
      break;
    }

    //choose the optimal neighbor. Pure virtual function.
    p = neighbors[ChooseOptimalNeighbor(_col, _c1, _c2, neighbors)];
    neighbors.clear();

    //chose new p so we need to detect cycles
    bool hasCycle = false;
    for(typename deque<CfgType>::iterator cit = hist.begin(); cit != hist.end(); cit++) {
      if(p == *cit) {
        hasCycle = true;
        break;
      }
    }

    //cycle has been detected, return false
    if(hasCycle) {
      connected = false;
      if(this->m_debug)
        cout << this->GetNameAndLabel() << ":: Local minima" << endl;
      break;
    }

    iter++;

    if(_savePath) {
      _lpOutput->m_path.push_back(p);
    }
    _lpOutput->m_intermediates.push_back(p);

    //too many tries have been attempted
    if((++tries > m_maxTries * nTicks)) {
      connected = false;
      if(this->m_debug)
        cout << this->GetNameAndLabel() << ":: Max tries reached" << endl;
      break;
    }
  } while(p != _c2);

  _lpOutput->m_path.push_back(p);
  _lpOutput->m_intermediates.push_back(p);
  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + iter);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + iter);

  if(connected)
    stats->IncLPConnections(this->GetNameAndLabel());

  return connected;
}

template <class MPTraits>
vector<typename MPTraits::CfgType>
AStar<MPTraits>::FindNeighbors(
    const CfgType& _current, const CfgType& _goal, const CfgType& _increment) {
  vector<CfgType> neighbors, ret;
  vector<double> posOnly, oriOnly;
  string callee = this->GetNameAndLabel() + "::FindNeighbors";
  ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
  StatClass* stats = this->GetMPProblem()->GetStatClass();

  //Push 2 cfgs into neighbors whose position or orientation is the same
  //as _increment
  CfgType tmp = _increment;
  neighbors.push_back(tmp);
  size_t i;
  for(i = 0; i < _current.DOF(); ++i) {
    if(i < _current.PosDOF()) {
      posOnly.push_back(_increment.GetData()[i]);
      oriOnly.push_back(0.0);
    }
    else {
      posOnly.push_back(0.0);
      oriOnly.push_back(_increment.GetData()[i]);
    }
  }
  CfgType posCfg;
  posCfg.SetData(posOnly);
  CfgType oriCfg;
  oriCfg.SetData(oriOnly);
  neighbors.push_back(posCfg);
  neighbors.push_back(oriCfg);

  /////////////////////////////////////////////////////////////////////
  //Push m_dof cfgs into neighbors whose value in each dimension is the same
  //as or complement of _increment

  // find close neighbour in every dimension.
  vector<double> oneDim;
  for(i = 0; i < _current.DOF(); i++)
    oneDim.push_back(0.0);
  CfgType oneDimCfg, oneDimCfgNegative;
  for(i = 0; i < _current.DOF(); i++) {
    oneDim[i] = _increment.GetData()[i];

    oneDimCfg.SetData(oneDim);
    neighbors.push_back(oneDimCfg);

    oneDimCfgNegative = (-oneDimCfg);
    neighbors.push_back(oneDimCfgNegative);

    oneDim[i] = 0.0;  // reset to 0.0
  }

  /////////////////////////////////////////////////////////////////////
  //Validate Neighbors
  int cdCounter = 0;
  for(size_t i = 0;i < neighbors.size(); ++i) {
    CfgType tmp = _current;
    tmp.IncrementTowardsGoal(_goal, neighbors[i]);
    if(_current == tmp) continue;
    cdCounter++;
    if(vc->IsValid(tmp, callee))
      ret.push_back(tmp);
    if(ret.size() >= m_numNeighbors)
      break;
  }
  stats->IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief A* like local planning which optimizes distance.
/// @tparam MPTraits Motion planning universe
///
/// Specialized A* like algorithm for distance based optimization.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class AStarDistance : public AStar<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    AStarDistance(const string& _vcLabel = "", const string& _dmLabel = "",
        size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5);

    AStarDistance(MPProblemType* _problem, XMLNode& _node);

    virtual ~AStarDistance();

    virtual void Print(ostream& _os) const;

    virtual size_t ChooseOptimalNeighbor(CfgType& _col,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors);

  private:
    string m_dmLabel;
};

template <class MPTraits>
AStarDistance<MPTraits>::
AStarDistance(const string& _vcLabel, const string& _dmLabel,
    size_t _maxTries, size_t _numNeighbors, size_t _histLength) :
  AStar<MPTraits>(_vcLabel, _maxTries, _numNeighbors, _histLength),
  m_dmLabel(_dmLabel) {
    this->SetName("AStarDistance");
  }

template <class MPTraits>
AStarDistance<MPTraits>::
AStarDistance(MPProblemType* _problem, XMLNode& _node) :
  AStar<MPTraits>(_problem, _node) {
    this->SetName("AStarDistance");
    m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric Label");
  }

template <class MPTraits>
AStarDistance<MPTraits>::~AStarDistance() {}

template <class MPTraits>
void
AStarDistance<MPTraits>::Print(ostream& _os) const {
  AStar<MPTraits>::Print(_os);
  _os << "\tdm label : " << m_dmLabel << endl;
}

//find Cfg closest to goal. ASTAR_DISTANCE
template <class MPTraits>
size_t
AStarDistance<MPTraits>::ChooseOptimalNeighbor(CfgType& _col,
    const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) {
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
  double minDistance = MAX_DBL;
  size_t retPosition = 0;
  double value = 0;
  for(size_t i = 0; i < _neighbors.size(); i++) {
    value = dm->Distance(_neighbors[i], _c2);
    if (value < minDistance) {
      retPosition = i;
      minDistance = value;
    }
  }
  return retPosition;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief A* like local planning which optimizes clearance.
/// @tparam MPTraits Motion planning universe
///
/// Specialized A* like algorithm for clearance based optimization.
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class AStarClearance : public AStar<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    AStarClearance(const string& _vcLabel = "",
        size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5,
        const ClearanceUtility<MPTraits>& _c = ClearanceUtility<MPTraits>());

    AStarClearance(MPProblemType* _problem, XMLNode& _node);

    virtual ~AStarClearance();

    virtual void Print(ostream& _os) const;

    virtual size_t ChooseOptimalNeighbor(CfgType& _col,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors);

  private:
    ClearanceUtility<MPTraits> m_clearanceUtility;
};

template <class MPTraits>
AStarClearance<MPTraits>::
AStarClearance(const string& _vcLabel,
    size_t _maxTries, size_t _numNeighbors, size_t _histLength,
    const ClearanceUtility<MPTraits>& _c) :
  AStar<MPTraits>(_vcLabel, _maxTries, _numNeighbors, _histLength), m_clearanceUtility(_c) {
    this->SetName("AStarClearance");
  }

template <class MPTraits>
AStarClearance<MPTraits>::
AStarClearance(MPProblemType* _problem, XMLNode& _node) :
  AStar<MPTraits>(_problem, _node), m_clearanceUtility(_problem, _node) {
    this->SetName("AStarClearance");
  }

template <class MPTraits>
AStarClearance<MPTraits>::~AStarClearance() {}

template <class MPTraits>
void
AStarClearance<MPTraits>::Print(ostream& _os) const {
  AStar<MPTraits>::Print(_os);
  m_clearanceUtility.Print(_os);
}

//find Cfg with largest clearance. ASTAR_CLEARANCE
template <class MPTraits>
size_t
AStarClearance<MPTraits>::ChooseOptimalNeighbor(CfgType& _col,
    const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) {
  Environment* env = this->GetMPProblem()->GetEnvironment();
  double maxClearance = -MAX_DBL;
  size_t retPosition = 0;
  double value = 0;

  CDInfo tmpInfo;
  CfgType tmp;

  for(size_t i = 0; i < _neighbors.size(); i++) {
    m_clearanceUtility.CollisionInfo(_neighbors[i], tmp, env->GetBoundary(), tmpInfo);
    value = tmpInfo.m_minDist;
    if (value > maxClearance) {
      retPosition = i;
      maxClearance = value;
    }
  }
  return retPosition;
}

#endif
