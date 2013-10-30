/**
 * AStar.h
 * Performs AStar Like Local planning, basic algorithm:
 *
 * while(goal not reached or tries < maxTries){
 *   find neighbors;
 *   step towards optimal neighbors;
 * }
 * if(goal reached) return connected;
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include "LocalPlannerMethod.h"
#include "Utilities/MPUtils.h"
#include "Utilities/MedialAxisUtilities.h"
#include "ValidityCheckers/CollisionDetection/CDInfo.h"

template <class MPTraits>
class AStar : public LocalPlannerMethod<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    AStar(string _vcLabel = "", size_t _maxTries = 0, size_t _numNeighbors = 0,
        size_t _histLength = 5);

    AStar(MPProblemType* _problem, XMLNodeReader& _node);

    virtual ~AStar();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    virtual vector<CfgType> ReconstructPath(
        Environment* _env, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, const vector<CfgType>& _intermediates,
        double _posRes, double _oriRes) {
      vector<CfgType> tmp = _intermediates;
      return tmp;
    }

  protected:
    bool SetLPOutputFail(const CfgType& _c, const CfgType& _p,
        LPOutput<MPTraits>* _lpOutput, string _debugMsg);

    virtual bool IsConnectedOneWay(
        Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision = true, bool _savePath = false, bool _saveFailedPath = false);

    virtual size_t ChooseOptimalNeighbor(
        Environment* _env, StatClass& _stats, CfgType& _col, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) = 0;

    virtual vector<CfgType> FindNeighbors(Environment* _env, StatClass& _stats,
        const CfgType& _current, const CfgType& _goal, const CfgType& _increment);

    string m_vcLabel;
    size_t m_maxTries;     // How many time will be tried to connect to goal
    size_t m_numNeighbors; // How many neighbors will be seached abound current Cfg
    size_t m_histLength;   // how many nodes should I keep track of for cycles
};


template <class MPTraits>
AStar<MPTraits>::AStar(string _vcLabel,
    size_t _maxTries, size_t _numNeighbors, size_t _histLength) :
    LocalPlannerMethod<MPTraits>(), m_vcLabel(_vcLabel), m_maxTries(_maxTries),
    m_numNeighbors(_numNeighbors), m_histLength(_histLength) {
  this->SetName("AStar");
}


template <class MPTraits>
AStar<MPTraits>::AStar(MPProblemType* _problem, XMLNodeReader& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node) {
  this->SetName("AStar");
  m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
  m_maxTries = _node.numberXMLParameter("maxTries", true, 0, 0, MAX_INT, "n tries");
  m_numNeighbors = _node.numberXMLParameter("numNeighbors", true, 0, 0, MAX_INT, "n neighbors");
  m_histLength = _node.numberXMLParameter("histLength", false, 5, 0, MAX_INT,
      "history length for detecting cycles");
}


template <class MPTraits>
AStar<MPTraits>::~AStar() {}


template <class MPTraits>
void
AStar<MPTraits>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetNameAndLabel() << "::  "
      << "maxTries = " << m_maxTries << " "
      << "numNeighbors = " << m_numNeighbors << " "
      << "vcLabel = " << m_vcLabel << " "
      << "histLength = " << m_histLength << " "
      << endl;
}


template <class MPTraits>
bool
AStar<MPTraits>::IsConnected(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  //clear _lpOutput
  _lpOutput->Clear();
  bool connected = false;

  connected = IsConnectedOneWay(_env, _stats, _dm, _c1, _c2,_col, _lpOutput,
      _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);

  if(!connected) { //try the other way
    connected = IsConnectedOneWay(_env, _stats, _dm, _c2, _c1,_col, _lpOutput,
        _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);
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
AStar<MPTraits>::SetLPOutputFail(const CfgType& _c, const CfgType& _p,
    LPOutput<MPTraits>* _lpOutput, string _debugMsg) {
  if(this->m_debug) {
    cout << this->GetNameAndLabel() << "::" << _debugMsg << endl;
  }
  pair<pair<CfgType,CfgType>, pair<WeightType,WeightType> > tmp;
  tmp.first.first = _c;
  tmp.first.second = _p;
  tmp.second.first = _lpOutput->m_edge.first;
  tmp.second.second = _lpOutput->m_edge.second;
  _lpOutput->m_savedEdge.push_back(tmp);
  return false;
}


template <class MPTraits>
bool
AStar<MPTraits>::IsConnectedOneWay(
    Environment* _env, StatClass& _stats, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, CfgType& _col,
    LPOutput<MPTraits>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  if(this->m_debug) {
    VDClearAll();
    VDAddTempCfg(_c1, false);
    VDAddTempCfg(_c2, false);
  }

  _stats.IncLPAttempts(this->GetNameAndLabel());

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
    neighbors = FindNeighbors(_env, _stats, p, _c2, incr);

    //neighbors all in collision
    if(neighbors.size() == 0) {
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Found 0 Neighbors");
      break;
    }

    //choose the optimal neighbor. Pure virtual function.
    p = neighbors[ChooseOptimalNeighbor(_env, _stats,_col, _dm, _c1, _c2, neighbors)];
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
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Local Minima");
      break;
    }

    iter++;

    if(_savePath || _saveFailedPath) {
      _lpOutput->m_path.push_back(p);
    }
    _lpOutput->m_intermediates.push_back(p);

    //too many tries have been attempted
    if((++tries > m_maxTries * nTicks)) {
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Max Tries Reached");
      break;
    }
  } while(p != _c2);

  _lpOutput->m_path.push_back(p);
  _lpOutput->m_intermediates.push_back(p);
  _lpOutput->m_edge.first.SetWeight(_lpOutput->m_edge.first.GetWeight() + iter);
  _lpOutput->m_edge.second.SetWeight(_lpOutput->m_edge.second.GetWeight() + iter);

  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  return connected;
}


template <class MPTraits>
vector<typename MPTraits::CfgType>
AStar<MPTraits>::FindNeighbors(
    Environment* _env, StatClass& _stats,
    const CfgType& _current, const CfgType& _goal, const CfgType& _increment) {
  vector<CfgType> neighbors, ret;
  vector<double> posOnly, oriOnly;
  string callee = this->GetNameAndLabel() + "::FindNeighbors";
  ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

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
    if(vcm->IsValid(tmp, callee)) {
      ret.push_back(tmp);
    }
    if(ret.size() >= m_numNeighbors)
      break;
  }
  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter);
  return ret;
}

//////////////////////////////////////////////////////////////////
// AStarDistance
//////////////////////////////////////////////////////////////////

template <class MPTraits>
class AStarDistance : public AStar<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    AStarDistance(string _vcLabel = "", size_t _maxTries = 0,
        size_t _numNeighbors = 0, size_t _histLength = 5);

    AStarDistance(MPProblemType* _problem, XMLNodeReader& _node);

    virtual ~AStarDistance();

    virtual size_t ChooseOptimalNeighbor(
        Environment* _env, StatClass& _stats, CfgType& _col, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors);
};


template <class MPTraits>
AStarDistance<MPTraits>::AStarDistance(
    string _vcLabel, size_t _maxTries, size_t _numNeighbors, size_t _histLength) :
    AStar<MPTraits>(_vcLabel, _maxTries, _numNeighbors, _histLength) {
  this->SetName("AStarDistance");
}


template <class MPTraits>
AStarDistance<MPTraits>::AStarDistance(MPProblemType* _problem, XMLNodeReader& _node) :
    AStar<MPTraits>(_problem, _node) {
  this->SetName("AStarDistance");
  _node.warnUnrequestedAttributes();
}


template <class MPTraits>
AStarDistance<MPTraits>::~AStarDistance() {}


//find Cfg closest to goal. ASTAR_DISTANCE
template <class MPTraits>
size_t
AStarDistance<MPTraits>::ChooseOptimalNeighbor(
    Environment* _env, StatClass& _stats, CfgType& _col, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) {
  double minDistance = MAX_DBL;
  size_t retPosition = 0;
  double value = 0;
  for(size_t i = 0; i < _neighbors.size(); i++) {
    value = _dm->Distance(_env, _neighbors[i], _c2);
    if (value < minDistance) {
      retPosition = i;
      minDistance = value;
    }
  }
  return retPosition;
}

//////////////////////////////////////////////////////////////////
// AStarClearance
//////////////////////////////////////////////////////////////////

template <class MPTraits>
class AStarClearance : public AStar<MPTraits> {
  public:

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;

    AStarClearance(string _vcLabel = "",
        size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5,
        const ClearanceUtility<MPTraits>& _c = ClearanceUtility<MPTraits>());

    AStarClearance(MPProblemType* _problem, XMLNodeReader& _node);

    virtual ~AStarClearance();

    virtual void PrintOptions(ostream& _os);

    virtual size_t ChooseOptimalNeighbor(
        Environment* _env, StatClass& _stats, CfgType& _col, DistanceMetricPointer _dm,
        const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors);

  private:
    ClearanceUtility<MPTraits> m_clearanceUtility;
};


template <class MPTraits>
AStarClearance<MPTraits>::AStarClearance(
    string _vcLabel,
    size_t _maxTries, size_t _numNeighbors, size_t _histLength,
    const ClearanceUtility<MPTraits>& _c) :
    AStar<MPTraits>(_vcLabel, _maxTries, _numNeighbors, _histLength), m_clearanceUtility(_c) {
  this->SetName("AStarClearance");
}


template <class MPTraits>
AStarClearance<MPTraits>::AStarClearance(MPProblemType* _problem, XMLNodeReader& _node) :
    AStar<MPTraits>(_problem, _node), m_clearanceUtility(_problem, _node) {
  this->SetName("AStarClearance");
  _node.warnUnrequestedAttributes();
}


template <class MPTraits>
AStarClearance<MPTraits>::~AStarClearance() {}


template <class MPTraits>
void
AStarClearance<MPTraits>::PrintOptions(ostream& _os) {
  AStar<MPTraits>::PrintOptions(_os);
  m_clearanceUtility.PrintOptions(_os);
}


//find Cfg with largest clearance. ASTAR_CLEARANCE
template <class MPTraits>
size_t
AStarClearance<MPTraits>::ChooseOptimalNeighbor(
    Environment* _env, StatClass& _stats, CfgType& _col, DistanceMetricPointer _dm,
    const CfgType& _c1, const CfgType& _c2, vector<CfgType>& _neighbors) {
  double maxClearance = -MAX_DBL;
  size_t retPosition = 0;
  double value = 0;

  CDInfo tmpInfo;
  CfgType tmp;

  for(size_t i = 0; i < _neighbors.size(); i++) {
    m_clearanceUtility.CollisionInfo(_neighbors[i], tmp, _env->GetBoundary(), tmpInfo);
    value = tmpInfo.m_minDist;
    if (value > maxClearance) {
      retPosition = i;
      maxClearance = value;
    }
  }
  return retPosition;
}

#endif
