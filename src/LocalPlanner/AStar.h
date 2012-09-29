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
#include "MPUtils.h"
#include "CDInfo.h"

template <class CFG, class WEIGHT>
class AStar: public LocalPlannerMethod<CFG, WEIGHT> {
  public:
    AStar(string _vcMethod = "", size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5);
    AStar(XMLNodeReader& _node, MPProblem* _problem, bool _warnUnrequestedXML = false);
    virtual ~AStar();

    virtual void PrintOptions(ostream& _os);

    virtual bool IsConnected(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes, bool _checkCollision = true, 
        bool _savePath = false, bool _saveFailedPath = false);

    virtual vector<CFG> ReconstructPath(Environment* _env, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, const vector<CFG>& _intermediates, 
        double _posRes, double _oriRes) {
      vector<CFG> tmp = _intermediates;
      return tmp;
    }

  protected:
    bool SetLPOutputFail(const CFG& _c, const CFG& _p, LPOutput<CFG, WEIGHT>* _lpOutput, string _debugMsg);

    virtual bool IsConnectedOneWay(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes, bool _checkCollision = true, 
        bool _savePath = false, bool _saveFailedPath = false);

    virtual size_t ChooseOptimalNeighbor(Environment* _env, StatClass& _stats,
        CFG& _col, shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2, vector<CFG>& _neighbors) = 0;

    virtual vector<CFG> FindNeighbors(Environment* _env, StatClass& _stats, 
        const CFG& _current, const CFG& _goal, const CFG& _increment);

    string m_vcMethod;
    size_t m_maxTries;     // How many time will be tried to connect to goal. (not used!?)
    size_t m_numNeighbors; // How many neighbors will be seached abound current Cfg. (not used?!) 
    size_t m_histLength; // how many nodes should I keep track of for cycles
};

template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::AStar(string _vcMethod, size_t _maxTries, size_t _numNeighbors, size_t _histLength) : LocalPlannerMethod<CFG, WEIGHT>(), 
  m_vcMethod(_vcMethod), m_maxTries(_maxTries), m_numNeighbors(_numNeighbors), m_histLength(_histLength) {
    this->SetName("AStar");
  }

template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::AStar(XMLNodeReader& _node, MPProblem* _problem, bool _warnUnrequestedXML) :
  LocalPlannerMethod<CFG,WEIGHT>(_node, _problem) {
    this->SetName("AStar");
    m_vcMethod = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
    m_maxTries = _node.numberXMLParameter("maxTries", true, 0, 0, 10, "n tries");
    m_numNeighbors = _node.numberXMLParameter("numNeighbors", true, 0, 0, 10, "n neighbors");
    m_histLength = _node.numberXMLParameter("histLength", false, 5, 0, MAX_INT, "history length for detecting cycles");
    if(_warnUnrequestedXML)
      _node.warnUnrequestedAttributes();
  }

template <class CFG, class WEIGHT> 
AStar<CFG, WEIGHT>::~AStar() {}

template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "maxTries" << " " <<m_maxTries << " ";
  _os << "numNeighbors" << " " <<m_numNeighbors << " ";
  _os << "vcMethod = " << " " << m_vcMethod << " ";
  _os << "histLength = " << " " << m_histLength << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::IsConnected(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes, bool _checkCollision, bool _savePath, bool _saveFailedPath) {
  //clear _lpOutput
  _lpOutput->Clear();
  bool connected = false;

  connected = IsConnectedOneWay(_env, _stats, _dm, _c1, _c2,_col, _lpOutput,
      _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);

  if (!connected) { //try the other way
    connected = IsConnectedOneWay(_env, _stats, _dm, _c2, _c1,_col, _lpOutput,
        _positionRes, _orientationRes, _checkCollision, _savePath, _saveFailedPath);

    if (_savePath)
      reverse(_lpOutput->path.begin(), _lpOutput->path.end());
  }
  if(connected){
    _lpOutput->AddIntermediatesToWeights(this->m_saveIntermediates);
    _lpOutput->SetLPLabel(this->GetLabel());
  }
  return connected;
}

template <class CFG, class WEIGHT>
bool
AStar<CFG, WEIGHT>::SetLPOutputFail(const CFG& _c, const CFG& _p, LPOutput<CFG, WEIGHT>* _lpOutput, string _debugMsg){
  if(this->m_debug){
    cout << this->GetNameAndLabel() << "::" << _debugMsg << endl;
  }
  pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
  tmp.first.first = _c;
  tmp.first.second = _p;
  tmp.second.first = _lpOutput->edge.first;
  tmp.second.second = _lpOutput->edge.second;
  _lpOutput->savedEdge.push_back(tmp);
  return false;
}

template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::IsConnectedOneWay(Environment* _env, StatClass& _stats, shared_ptr<DistanceMetricMethod> _dm, 
    const CFG& _c1, const CFG& _c2, CFG& _col, LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes, bool _checkCollision, bool _savePath, bool _saveFailedPath) {

  if(this->m_debug){
    VDClearAll();
    VDAddTempCfg(_c1, false);
    VDAddTempCfg(_c2, false);
  }

  _stats.IncLPAttempts(this->GetNameAndLabel());

  CFG p = _c1;
  CFG incr;
  vector<CFG> neighbors;
  int nTicks;
  bool connected = true;
  size_t tries = 0;
  size_t iter = 0;
  deque<CFG> hist; //hist for detecting cycles

  incr.FindIncrement(_c1, _c2, &nTicks, _positionRes, _orientationRes);

  do {
    if(this->m_debug)
      VDAddTempCfg(p, true);

    //update cycle history
    hist.push_front(p);
    if(hist.size() > m_histLength) hist.pop_back();

    //find neighbors
    neighbors = FindNeighbors(_env, _stats, p, _c2, incr);    
    
    //neighbors all in collision
    if (neighbors.size()==0) {
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Found 0 Neighbors");
      break;
    }

    //choose the optimal neighbor. Pure virtual function.
    p = neighbors[ChooseOptimalNeighbor(_env, _stats,_col, _dm, _c1, _c2, neighbors)];
    neighbors.clear();

    //chose new p so we need to detect cycles
    bool hasCycle = false;
    for(typename deque<CFG>::iterator cit = hist.begin(); cit!=hist.end(); cit++){
      if(p==*cit){
        hasCycle = true;
        break;
      }
    }
    //cycle has been detected, return false
    if(hasCycle){
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Local Minima");
      break;
    }

    iter++;   

    if(_savePath || _saveFailedPath) {
      _lpOutput->path.push_back(p);
    }
    _lpOutput->intermediates.push_back(p);
    //too many tries have been attempted
    if ((++tries> m_maxTries * nTicks)) {
      connected = SetLPOutputFail(_c1, p, _lpOutput, "Max Tries Reached");
      break;
    } ;
  } while(p!=_c2);

  _lpOutput->path.push_back(p);
  _lpOutput->intermediates.push_back(p);
  _lpOutput->edge.first.SetWeight(_lpOutput->edge.first.GetWeight() + iter);
  _lpOutput->edge.second.SetWeight(_lpOutput->edge.second.GetWeight() + iter);

  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel());

  return connected;
};

template <class CFG, class WEIGHT>
vector<CFG> 
AStar<CFG, WEIGHT>::FindNeighbors(Environment* _env, StatClass& _stats, const CFG& _current, const CFG& _goal, const CFG& _increment) {
  vector<CFG> neighbors, ret;  
  vector<double> posOnly, oriOnly;
  string callee = this->GetNameAndLabel()+"::FindNeighbors";
  CDInfo cdInfo;
  typename ValidityChecker::ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker()->GetMethod(m_vcMethod);
  
  //Push 2 cfgs into neighbors whose position or orientation is the same 
  //as _increment
  CFG tmp = _increment;
  neighbors.push_back(tmp);
  size_t i;
  for(i=0; i<_current.DOF(); ++i) {
    if(i<_current.PosDOF()) {
      posOnly.push_back(_increment.GetData()[i]);
      oriOnly.push_back(0.0);
    } 
    else {
      posOnly.push_back(0.0);
      oriOnly.push_back(_increment.GetData()[i]);
    }
  }
  CFG posCfg; 
  posCfg.SetData(posOnly);
  CFG oriCfg;
  oriCfg.SetData(oriOnly);
  neighbors.push_back(posCfg);
  neighbors.push_back(oriCfg);

  /////////////////////////////////////////////////////////////////////
  //Push m_dof cfgs into neighbors whose value in each dimension is the same
  //as or complement of _increment

  // find close neighbour in every dimension.
  vector<double> oneDim;
  for(i=0; i< _current.DOF(); i++)
    oneDim.push_back(0.0);
  CFG oneDimCfg, oneDimCfgNegative;
  for(i=0; i< _current.DOF(); i++) {
    oneDim[i] = _increment.GetData()[i];

    oneDimCfg.SetData(oneDim);
    neighbors.push_back(oneDimCfg);

    oneDimCfgNegative.negative(oneDimCfg);
    neighbors.push_back(oneDimCfgNegative);

    oneDim[i] = 0.0;  // reset to 0.0
  }

  /////////////////////////////////////////////////////////////////////
  //Validate Neighbors
  int cdCounter = 0;
  for(size_t i=0;i<neighbors.size();++i) {
    CFG tmp = _current;
    tmp.IncrementTowardsGoal(_goal, neighbors[i]);
    if(_current==tmp) continue;
    cdCounter++;
    if(vcm->IsValid(tmp, _env, _stats, cdInfo, &callee) ) {
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

template <class CFG, class WEIGHT>
class AStarDistance: public AStar<CFG, WEIGHT> {
  public:
    AStarDistance(string _vcMethod = "", size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5);
    AStarDistance(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~AStarDistance();

    virtual size_t ChooseOptimalNeighbor(Environment* _env, StatClass& _stats,
        CFG& _col, shared_ptr<DistanceMetricMethod> _dm,
        const CFG& _c1, const CFG& _c2, vector<CFG>& _neighbors); 
};

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::AStarDistance(string _vcMethod, size_t _maxTries, size_t _numNeighbors, size_t _histLength) : 
  AStar<CFG,WEIGHT>(_vcMethod, _maxTries, _numNeighbors, _histLength) {
    this->SetName("AStarDistance");
  }

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::AStarDistance(XMLNodeReader& _node, MPProblem* _problem) :
  AStar<CFG,WEIGHT>(_node, _problem, true) {
    this->SetName("AStarDistance");
  }

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::~AStarDistance() {}

//find Cfg closest to goal. ASTAR_DISTANCE
template <class CFG, class WEIGHT>
size_t
AStarDistance<CFG, WEIGHT>::ChooseOptimalNeighbor(Environment* _env, StatClass& _stats,
    CFG& _col, shared_ptr<DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, vector<CFG>& _neighbors) {
  double minDistance = MAXFLOAT;
  size_t retPosition = 0;
  double value = 0;
  for(size_t i=0;i<_neighbors.size();i++) {
    value = _dm->Distance(_env, _neighbors[i], _c2);
    if (value<minDistance) {
      retPosition=i;
      minDistance=value;
    }    
  }
  return retPosition;
}

//////////////////////////////////////////////////////////////////
// AStarClearance
//////////////////////////////////////////////////////////////////

template <class CFG, class WEIGHT>
class AStarClearance: public AStar<CFG, WEIGHT> {
  public:
    AStarClearance(string _vcMethod = "", size_t _maxTries = 0, size_t _numNeighbors = 0, size_t _histLength = 5, size_t _penetration = 0);
    AStarClearance(XMLNodeReader& _node, MPProblem* _problem);

    virtual ~AStarClearance();

    virtual size_t ChooseOptimalNeighbor(Environment* _env, StatClass& _stats,
        CFG& _col, shared_ptr<DistanceMetricMethod> _dm, 
        const CFG& _c1, const CFG& _c2, vector<CFG>& _neighbors); 

  private:
    size_t m_penetration;
};

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::AStarClearance(string _vcMethod, size_t _maxTries, size_t _numNeighbors, size_t _histLength, size_t _penetration) : 
  AStar<CFG,WEIGHT>(_vcMethod, _maxTries, _numNeighbors, _histLength), m_penetration(_penetration) {
    this->SetName("AStarClearance");
  }

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::AStarClearance(XMLNodeReader& _node, MPProblem* _problem) : 
  AStar<CFG,WEIGHT>(_node,_problem,false) {
    this->SetName("AStarClearance");
    m_penetration = _node.numberXMLParameter("penetration", false, 5, 0, 1000, "Penetration Number");
    _node.warnUnrequestedAttributes();
  }

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::~AStarClearance() {}

//find Cfg with largest clearance. ASTAR_CLEARANCE
template <class CFG, class WEIGHT>
size_t
AStarClearance<CFG, WEIGHT>::ChooseOptimalNeighbor(Environment* _env, StatClass& _stats,
    CFG& _col, shared_ptr<DistanceMetricMethod> _dm,
    const CFG& _c1, const CFG& _c2, vector<CFG>& _neighbors) {
  double maxClearance = -MAXFLOAT;
  size_t retPosition = 0;
  double value = 0;
  MPProblem* mp = this->GetMPProblem();

  CDInfo tmpInfo;
  CfgType tmp;
  for(size_t i = 0; i < _neighbors.size(); i++) {
    GetApproxCollisionInfo(mp,_neighbors[i],tmp,_env,_stats,tmpInfo,this->m_vcMethod,
        _dm->GetLabel(), m_penetration, m_penetration, true, true);

    value = tmpInfo.m_minDist;
    if (value > maxClearance) {
      retPosition = i;
      maxClearance = value;
    }   
  }
  return retPosition;
}

#endif
