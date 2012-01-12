/**
 * AStar.h
 * Performs AStar Local planning
 *
 * Last Updated : 11/15/11
 * Update Author: Kasra Manavi
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
  AStar(XMLNodeReader& in_Node, MPProblem* in_pProblem);
  virtual ~AStar();
  //@}

  virtual void PrintOptions(ostream& out_os);
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
    virtual bool IsConnected(Environment *env, StatClass& Stats,
      shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
      LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
      bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);


 protected:
  virtual bool IsConnectedOneWay(Environment *env, StatClass& Stats,
    shared_ptr<DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, CFG &_col, 
    LPOutput<CFG, WEIGHT>* lpOutput, double positionRes, double orientationRes,
    bool checkCollision=true, bool savePath=false, bool saveFailedPath=false);

  virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& Stats, //CollisionDetection *cd,
    CFG &_col,shared_ptr< DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2,vector<Cfg*> &neighbors);

  int    n_tries;     // How many time will be tried to connect to goal. (not used!?)
  int    n_neighbors; // How many neighbors will be seached abound current Cfg. (not used?!) 
  cd_predefined  cdtype;
  std::string vcMethod;
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
AStar<CFG, WEIGHT>::AStar(XMLNodeReader& in_Node, MPProblem* in_pProblem) :
LocalPlannerMethod<CFG,WEIGHT>(in_Node,in_pProblem) {
  this->SetName("AStar");
  vcMethod = in_Node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
  n_tries = in_Node.numberXMLParameter("n_tries", true, 0, 0, 10, "n_tries");
  n_neighbors = in_Node.numberXMLParameter("n_neighbors", true, 0, 0, 10, "n_neighbors");
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
int AStar<CFG, WEIGHT>::ChooseOptimalNeighbor(Environment *_env, StatClass& Stats, CFG &_col,
                                              shared_ptr<DistanceMetricMethod > dm, 
                                              const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) {
  double minDistance= MAXFLOAT;
  int retPosition=0;
  double value = 0;
  for(size_t i=0;i<neighbors.size();i++) {
    value=dm->Distance(_env,*neighbors[i],_c2);
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
PrintOptions(ostream& out_os) {
  out_os << "    " << this->GetName() << "::  ";
  out_os << "n_tries" << " " <<n_tries << " ";
  out_os << "n_neighbors" << " " <<n_neighbors << " ";
  out_os << "vcMethod = " << " " << vcMethod << " ";
  out_os << endl;
}
 

template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnected(Environment *_env, StatClass& Stats, 
	    shared_ptr< DistanceMetricMethod >dm, 
	    const CFG &_c1, const CFG &_c2,CFG &_col,LPOutput<CFG, WEIGHT>* lpOutput,
	    double positionRes, double orientationRes,
	    bool checkCollision, 
	    bool savePath, bool saveFailedPath) {
  //clear lpOutput
  lpOutput->path.clear();
  lpOutput->edge.first.SetWeight(0);
  lpOutput->edge.second.SetWeight(0);
  lpOutput->savedEdge.clear();
  bool connected = false;
  
  connected = IsConnectedOneWay(_env, Stats, dm, _c1, _c2,_col, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
  if (!connected) { //try the other way
    connected = IsConnectedOneWay(_env, Stats, dm, _c2, _c1,_col, lpOutput, positionRes, orientationRes, checkCollision, savePath, saveFailedPath);
    
    if (savePath)
      reverse(lpOutput->path.begin(), lpOutput->path.end());
  }
  
  return connected;
}


template <class CFG, class WEIGHT>
bool
AStar<CFG,WEIGHT>::
IsConnectedOneWay(Environment *_env, StatClass& Stats,
		 shared_ptr< DistanceMetricMethod>dm, 
		  const CFG &_c1, const CFG &_c2,CFG &_col,LPOutput<CFG, WEIGHT>* lpOutput,
		  double positionRes, double orientationRes,
		  bool checkCollision, 
		  bool savePath, bool saveFailedPath) {
  Stats.IncLPAttempts( "AStar" );
  int cd_cntr = 0;
  
  CFG p;
  p = _c1;
  CFG incr;
  incr = _c1; 
  CFG diagonal;
  diagonal = _c1;
  vector<Cfg*> neighbors;
  ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
  typename ValidityChecker<CFG>::VCMethodPtr vcm = vc->GetVCMethod(vcMethod);
  int n_ticks;
  bool connected = true;
  int noTries=0;
  int nTicks = 0;
  CDInfo cdInfo;
  
  std::string Callee, Method("-AStar::IsConnectedOneWay()");
    std::string tmpStr = Callee+Method;
  
  incr.FindIncrement(_c1,_c2,&n_ticks,positionRes,orientationRes);
  connected = vc->IsValid(vcm, incr, _env, Stats, cdInfo, true, &tmpStr);
  

  do {
    /* First check the diagonal to find out if it it available */
    diagonal = p;
    diagonal.IncrementTowardsGoal(_c2,incr);
  
    cd_cntr++;
    Callee=diagonal.GetName();
    
    if(diagonal.InBoundingBox(_env)&& !vc->IsValid(vcm, diagonal, _env, Stats, cdInfo, true, &tmpStr)){
     
      p = diagonal;
      connected = false;
    } else {
      
      neighbors.clear();
      p.FindNeighbors(this->GetMPProblem(), _env, Stats, _c2, incr, tmpStr, n_neighbors, cdInfo, neighbors);  // what is the replacement for this function. I'm thinking it should be getneighbors
  
      if (neighbors.size()==0) { 
	connected = false;
        pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
	tmp.first.first = _c1;
	tmp.first.second = p;
	tmp.second.first = lpOutput->edge.first;
	tmp.second.second = lpOutput->edge.second;
	lpOutput->savedEdge.push_back(tmp);
	break;
      }
      p = *(neighbors[ ChooseOptimalNeighbor(_env, Stats,_col, dm, _c1, _c2, neighbors) ]);
     
  
     //double values;
    

     }
   
    nTicks++;   
    
    if(savePath || saveFailedPath) {
      lpOutput->path.push_back(p);
    }
    
   if ((++noTries> 6 * n_ticks)) { //if num_of_try > total_ticks*6->give up
      connected = false;
     
      pair< pair<CFG,CFG>, pair<WEIGHT,WEIGHT> > tmp;
      tmp.first.first = _c1;
      tmp.first.second = p;
      tmp.second.first = lpOutput->edge.first;
      tmp.second.second = lpOutput->edge.second;
      lpOutput->savedEdge.push_back(tmp);
      break;
      
    }
  
    
  } while(!p.AlmostEqual(_c2));
   lpOutput->path.push_back(p);
 
			 
  lpOutput->edge.first.SetWeight(lpOutput->edge.first.GetWeight() + nTicks);
  lpOutput->edge.second.SetWeight(lpOutput->edge.second.GetWeight() + nTicks);
    
  Stats.IncLPCollDetCalls("AStar", cd_cntr );
  
  if(connected)
    Stats.IncLPConnections( "AStar" );
  
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
    AStarDistance(cd_predefined _cdtype);
    AStarDistance(cd_predefined _cdtype,int n_tries,int n_neighbors);
    AStarDistance(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml = true);

    virtual ~AStarDistance();

    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& Stats,
        CFG &_col, shared_ptr<DistanceMetricMethod >dm,
        const CFG &_c1, const CFG &_c2,
        vector<Cfg*> &neighbors); 
};
 
template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance(cd_predefined _cdtype) : AStar<CFG, WEIGHT>(_cdtype) {
  this->SetName("a_star_distance");
  AStar<CFG, WEIGHT>::SetDefault();
}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
 AStarDistance(cd_predefined _cdtype,int n_tries,int n_neighbors):AStar<CFG,WEIGHT>( _cdtype, n_tries,n_neighbors) {
  this->SetName("a_star_distance");
}

template <class CFG, class WEIGHT>
AStarDistance<CFG, WEIGHT>::
AStarDistance(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml ):
     AStar<CFG,WEIGHT>(_cdtype,in_Node,in_pProblem,false)
{
  this->SetName("a_star_distance");
  this->cdtype= _cdtype;
  if(warnUnrequestedXml)
    in_Node.warnUnrequestedAttributes();
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
ChooseOptimalNeighbor(Environment *_env, StatClass& Stats, CFG &_col,shared_ptr< DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) {
  double minDistance= MAXFLOAT;
  int retPosition=0;
  double value = 0;
 for(size_t i=0;i<neighbors.size();i++) {
    value=dm->Distance(_env,*neighbors[i],_c2);
    
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
    AStarClearance(cd_predefined _cdtype);
    AStarClearance(cd_predefined _cdtype,int n_tries,int n_neighbors);
    AStarClearance(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml = true);

    virtual ~AStarClearance();

    virtual LocalPlannerMethod<CFG, WEIGHT>* CreateCopy();

    virtual int ChooseOptimalNeighbor(Environment *_env, StatClass& Stats,
        CFG &_col,
        shared_ptr<DistanceMetricMethod >dm, 
        const CFG &_c1, const CFG &_c2,
        vector<Cfg*> &neighbors); 

    std::string dm_label;
    string vcMethod;
    int penetration;
};

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance(cd_predefined _cdtype) : AStar<CFG, WEIGHT>(_cdtype) {
  this->SetName("aStarClearance");
}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
 AStarClearance(cd_predefined _cdtype,int n_tries,int n_neighbors):AStar<CFG,WEIGHT>( _cdtype, n_tries,n_neighbors) {
  this->SetName("aStarClearance");
}

template <class CFG, class WEIGHT>
AStarClearance<CFG, WEIGHT>::
AStarClearance(cd_predefined _cdtype, XMLNodeReader& in_Node, MPProblem* in_pProblem, bool warnUnrequestedXml ):
     AStar<CFG,WEIGHT>(_cdtype,in_Node,in_pProblem,false)
{
  this->SetName("aStarClearance");
  this->cdtype= _cdtype;
  dm_label = in_Node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
  vcMethod = in_Node.stringXMLParameter(string("vc_method"), true, string(""), string("Validity Test Method"));
  penetration = in_Node.numberXMLParameter("penetration", false, 5, 0, 1000, "Penetration Number");
  if(warnUnrequestedXml)
    in_Node.warnUnrequestedAttributes();
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
ChooseOptimalNeighbor(Environment *_env, StatClass& Stats,CFG &_col,shared_ptr< DistanceMetricMethod >dm, const CFG &_c1, const CFG &_c2, vector<Cfg*> &neighbors) {
  double maxClearance=-MAXFLOAT;
  size_t retPosition=0;
  double value = 0;
  MPProblem *mp = this->GetMPProblem();
	CDInfo tmp_info;
  for(size_t i=0;i<neighbors.size();i++) {
		GetApproxCollisionInfo(mp,*((CfgType*)neighbors[i]),_env,Stats,tmp_info,vcMethod,dm_label,penetration,penetration,true);
		value = tmp_info.min_dist;
    if (value>maxClearance) {
      retPosition=i;
      maxClearance=value;
    }   
  }
  return retPosition;
}

#endif
