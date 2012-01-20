/**
 * AStar.h
 * Performs AStar Local planning
 *
 * Last Updated : 01/10/12
 * Update Author: Aditya Mahadevan
 */
#ifndef ASTAR_H_
#define ASTAR_H_

#include "LocalPlannerMethod.h"
#include "MPUtils.h"

template <class CFG, class WEIGHT>
class AStar: public LocalPlannerMethod<CFG, WEIGHT> {
  public:

    /** @name Constructors and Destructor */
    //@{
    AStar();
    AStar(XMLNodeReader& _node, MPProblem* _problem);
    virtual ~AStar();
    //@}

    virtual void PrintOptions(ostream& _os);
    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    /**
     * Roughly check if two Cfgs could be connected using clearance.
     * Algorithm is given here:
     *   -# set clearance1 as clearance for _c1
     *   -# set clearance2 as clearance for _c2
     *   -# set dist as distance from _c1 to c2
     *   -# if clearance1+clearance2 > dist
     *       -# connected
     *   -# else
     *       -# not connected
     *
     * @see Cfg::ApproxCSpaceClearance and Cfg::Clearance
     */
    virtual bool IsConnected(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);


  protected:
    virtual bool IsConnectedOneWay(Environment *_env, StatClass& _stats,
        shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        CFG &_col, LPOutput<CFG, WEIGHT>* _lpOutput,
        double _positionRes, double _orientationRes,
        bool _checkCollision=true,
        bool _savePath=false,
        bool _saveFailedPath=false);

    virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& _stats,
        CFG &_col,shared_ptr< DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        vector<Cfg*> &_neighbors);

    int m_nTries;     // How many time will be tried to connect to goal. (not used!?)
    int m_nNeighbors; // How many neighbors will be seached abound current Cfg. (not used?!) 
    string m_vcMethod;
};

/////////////////////////////////////////////////////////////////////
//
//  definitions for class StraightLine declarations
//
/////////////////////////////////////////////////////////////////////


template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::
AStar() : LocalPlannerMethod<CFG, WEIGHT>() {
  this->SetName("AStar");
}

template <class CFG, class WEIGHT>
AStar<CFG, WEIGHT>::
AStar(XMLNodeReader& _node, MPProblem* _problem) :
  LocalPlannerMethod<CFG,WEIGHT>(_node,_problem) {
    this->SetName("AStar");
    m_vcMethod = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
    m_nTries = _node.numberXMLParameter("n_tries", true, 0, 0, 10, "n_tries");
    m_nNeighbors = _node.numberXMLParameter("n_neighbors", true, 0, 0, 10, "n_neighbors");
  }

template <class CFG, class WEIGHT> AStar<CFG, WEIGHT>::~AStar() {}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
AStar<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new AStar<CFG, WEIGHT>(*this);
  return _copy;
}

//find Cfg closest to goal. ASTAR_DISTANCE                                                                                                 
template <class CFG, class WEIGHT> 
int AStar<CFG, WEIGHT>::
ChooseOptimalNeighbor(Environment *_env,
    StatClass& _stats, CFG &_col,
    shared_ptr<DistanceMetricMethod > _dm,
    const CFG &_c1, const CFG &_c2,
    vector<Cfg*> &_neighbors) {

  double minDistance= MAXFLOAT;
  int retPosition=0;
  double value = 0;
  for(size_t i=0;i<_neighbors.size();i++) {
    value=_dm->Distance(_env,*_neighbors[i],_c2);
    if (value<minDistance) {
      retPosition=i;
      minDistance=value;
    }
  }
  return retPosition;
}

template <class CFG, class WEIGHT>
void
AStar<CFG, WEIGHT>::
PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "nTries" << " " <<m_nTries << " ";
  _os << "m_nNeighbors" << " " <<m_nNeighbors << " ";
  _os << "m_vcMethod = " << " " << m_vcMethod << " ";
  _os << endl;
}

template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnected(Environment *_env, StatClass& _stats, 
    shared_ptr< DistanceMetricMethod >_dm, 
    const CFG &_c1, const CFG &_c2,CFG &_col,LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {
  //clear _lpOutput
  _lpOutput->path.clear();
  _lpOutput->edge.first.SetWeight(0);
  _lpOutput->edge.second.SetWeight(0);
  _lpOutput->savedEdge.clear();
  bool connected = false;

  connected = IsConnectedOneWay(_env, _stats, _dm,
      _c1, _c2,_col, _lpOutput,
      _positionRes, _orientationRes,
      _checkCollision, _savePath, _saveFailedPath);
  if (!connected) { //try the other way
    connected = IsConnectedOneWay(_env, _stats, _dm,
        _c2, _c1,_col, _lpOutput,
        _positionRes, _orientationRes,
        _checkCollision, _savePath, _saveFailedPath);

    if (_savePath)
      reverse(_lpOutput->path.begin(), _lpOutput->path.end());
  }

  return connected;
}


template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, StatClass& _stats,
    shared_ptr< DistanceMetricMethod>_dm, 
    const CFG &_c1, const CFG &_c2,CFG &_col,LPOutput<CFG, WEIGHT>* _lpOutput,
    double _positionRes, double _orientationRes,
    bool _checkCollision, 
    bool _savePath, bool _saveFailedPath) {
  _stats.IncLPAttempts( this->GetNameAndLabel() );
  int cdCounter = 0;

  CFG p;
  p = _c1;
  CFG incr;
  incr = _c1; 
  CFG diagonal;
  diagonal = _c1;
  vector<Cfg*> neighbors;
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(m_vcMethod);
  int nTicks;
  bool connected = true;
  int nTries=0;
  int nIter = 0;
  CDInfo cdInfo;

  string callee;
  string method = "-AStar::IsConnectedOneWay()";
  string tmpStr = callee+method;

  incr.FindIncrement(_c1,_c2,&nTicks,_positionRes,_orientationRes);
  connected = vc->IsValid(vcm, incr, _env, _stats, cdInfo, true, &tmpStr);


  do {
    /* First check the diagonal to find out if it it available */
    diagonal = p;
    diagonal.IncrementTowardsGoal(_c2,incr);

    cdCounter++;
    callee=diagonal.GetName();

    if(diagonal.InBoundingBox(_env)&& !vc->IsValid(vcm, diagonal, _env, _stats, cdInfo, true, &tmpStr)){

      p = diagonal;
      connected = false;
    } else {

      neighbors.clear();
      p.FindNeighbors(this->GetMPProblem(), _env, _stats, _c2, incr, tmpStr, m_nNeighbors, cdInfo, neighbors);    
      if (neighbors.size()==0) { 
        connected = false;
        pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
        tmp.first.first = _c1;
        tmp.first.second = p;
        tmp.second.first = _lpOutput->edge.first;
        tmp.second.second = _lpOutput->edge.second;
        _lpOutput->savedEdge.push_back(tmp);
        break;
      }
      p = *(neighbors[ ChooseOptimalNeighbor(_env, _stats,_col, _dm, _c1, _c2, neighbors) ]);

    }

    nIter++;   

    if(_savePath || _saveFailedPath) {
      _lpOutput->path.push_back(p);
    }

    if ((++nTries> 6 * nTicks)) { //if num_of_try > total_ticks*6->give up
      connected = false;

      pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
      tmp.first.first = _c1;
      tmp.first.second = p;
      tmp.second.first = _lpOutput->edge.first;
      tmp.second.second = _lpOutput->edge.second;
      _lpOutput->savedEdge.push_back(tmp);
      break;

    }


  } while(!p.AlmostEqual(_c2));
  _lpOutput->path.push_back(p);


  _lpOutput->edge.first.SetWeight(_lpOutput->edge.first.GetWeight() + nIter);
  _lpOutput->edge.second.SetWeight(_lpOutput->edge.second.GetWeight() + nIter);

  _stats.IncLPCollDetCalls(this->GetNameAndLabel(), cdCounter );

  if(connected)
    _stats.IncLPConnections(this->GetNameAndLabel() );

  for(size_t i=0; i<neighbors.size();i++) {
    if (neighbors[i] != NULL)
      delete neighbors[i];
  }


  return connected;
};

//
// AStarDistance

template <class CFG, class WEIGHT>
class AStarDistance: public AStar<CFG, WEIGHT> {
  public:
    //  AStarDistance();
    AStarDistance();
    AStarDistance(int _nTries,int _nNeighbors);
    AStarDistance(XMLNodeReader& _node, MPProblem* _problem, bool _warnUnrequestedXml = true);

    virtual ~AStarDistance();

    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& _stats,
        CFG &_col, shared_ptr<DistanceMetricMethod >_dm,
        const CFG &_c1, const CFG &_c2,
        vector<Cfg*> &_neighbors); 
};

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance() : AStar<CFG, WEIGHT>() {
  this->SetName("AStarDistance");
}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance(int _nTries,int _nNeighbors):
AStar<CFG,WEIGHT>(_nTries,_nNeighbors) {
  this->SetName("AStarDistance");
}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance(XMLNodeReader& _node,
    MPProblem* _problem,
    bool _warnUnrequestedXml ):
AStar<CFG,WEIGHT>(_node,_problem,false) {

  this->SetName("AStarDistance");
  if(_warnUnrequestedXml)
    _node.warnUnrequestedAttributes();
}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
~AStarDistance() {
}


template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
AStarDistance<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new AStarDistance<CFG, WEIGHT>(*this);
  return _copy;
}


//find Cfg closest to goal. ASTAR_DISTANCE
template <class CFG, class WEIGHT>
int
AStarDistance<CFG, WEIGHT>::
ChooseOptimalNeighbor(Environment *_env, StatClass& _stats,
    CFG &_col,shared_ptr< DistanceMetricMethod >_dm,
    const CFG &_c1, const CFG &_c2,
    vector<Cfg*> &_neighbors) {

  double minDistance= MAXFLOAT;
  int retPosition=0;
  double value = 0;
  for(size_t i=0;i<_neighbors.size();i++) {
    value=_dm->Distance(_env,*_neighbors[i],_c2);

    if (value<minDistance) {
      retPosition=i;
      minDistance=value;

    }    
  }

  return retPosition;
}


//
// AStarClearance

template <class CFG, class WEIGHT>
class AStarClearance: public AStar<CFG, WEIGHT> {
  public:
    AStarClearance();
    AStarClearance(int _nTries,int _nNeighbors);
    AStarClearance(XMLNodeReader& _node, MPProblem* _problem, bool _warnUnrequestedXml = true);

    virtual ~AStarClearance();

    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& _stats,
        CFG &_col,
        shared_ptr<DistanceMetricMethod >_dm, 
        const CFG &_c1, const CFG &_c2,
        vector<Cfg*> &_neighbors); 

    string m_dm;
    int m_penetration;
};

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance() : AStar<CFG, WEIGHT>() {
  this->SetName("AStarClearance");
}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance(int _nTries, int _nNeighbors):
AStar<CFG,WEIGHT>(_nTries,_nNeighbors) {
  this->SetName("AStarClearance");
}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance(XMLNodeReader& _node, MPProblem* _problem,
bool _warnUnrequestedXml ):
AStar<CFG,WEIGHT>(_node,_problem,false) {
  this->SetName("AStarClearance");
  m_dm = _node.stringXMLParameter("dm_method", true,
            "default", "Distance Metric Method");
  this->m_vcMethod = _node.stringXMLParameter("vc_method", true,
            "", "Validity Test Method");
  m_penetration = _node.numberXMLParameter("penetration", false,
            5, 0, 1000, "Penetration Number");
  if(_warnUnrequestedXml)
    _node.warnUnrequestedAttributes();
}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
~AStarClearance() {

}

template <class CFG, class WEIGHT>
LocalPlannerMethod<CFG, WEIGHT>* 
AStarClearance<CFG, WEIGHT>::
CreateCopy() {
  LocalPlannerMethod<CFG, WEIGHT> * _copy = new AStarClearance<CFG, WEIGHT>(*this);
  return _copy;
}

//find Cfg with largest clearance. ASTAR_CLEARANCE
template <class CFG, class WEIGHT>
int
AStarClearance<CFG, WEIGHT>::
ChooseOptimalNeighbor(Environment *_env, StatClass& _stats,
    CFG &_col,shared_ptr< DistanceMetricMethod >_dm,
    const CFG &_c1, const CFG &_c2,
    vector<Cfg*> &_neighbors) {

  double maxClearance=-MAXFLOAT;
  size_t retPosition=0;
  double value = 0;
  MPProblem *mp = this->GetMPProblem();
  CDInfo tmpInfo;
  for(size_t i=0;i<_neighbors.size();i++) {
    GetApproxCollisionInfo(mp,*((CfgType*)_neighbors[i]),_env,_stats,tmpInfo,this->m_vcMethod,m_dm,m_penetration,m_penetration,true);
    value = tmpInfo.min_dist;
    if (value>maxClearance) {
      retPosition=i;
      maxClearance=value;
    }   
  }
  return retPosition;
}

#endif
