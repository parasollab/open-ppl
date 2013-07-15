/**
 * OBRRTStrategy.h
 * 
 * Description: OBRRT Strategy header file
 *
 * Author: Evan Greco
 * Last Edited: 04/04/2011
 */

#ifndef OBRRTSTRATEGY_H_
#define OBRRTSTRATEGY_H_

#include "BasicRRTStrategy.h"

template<class MPTraits>
class OBRRTStrategy : public BasicRRTStrategy<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;

    OBRRTStrategy();
    OBRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node);
    virtual ~OBRRTStrategy() {}

    virtual void ParseXML(XMLNodeReader& _node);

  protected:
    // Helper functions
    VID ExpandTree(CfgType& _dir);
    CfgType g0(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g1(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g2(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri=false);
    //g3 is the same as g2 but with same orientation
    CfgType g4(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g5(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri=false);
    //g6 is the same as g5 but with same orientation
    CfgType g7(CfgType& _near, CfgType& _dir, bool& _verifiedValid);
    CfgType g8(CfgType& _near, CfgType& _dir, bool& _verifiedValid);

  private:
    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    double m_g0, m_g1, m_g2, m_g3, m_g4, m_g5, m_g6, m_g7, m_g8; //Growth method probabilities; can be found in OBRRT paper/TR
    double m_g0N, m_g1N, m_g2N, m_g3N, m_g4N, m_g5N, m_g6N, m_g7N, m_g8N; 
};

template<class MPTraits>
OBRRTStrategy<MPTraits>::OBRRTStrategy() {
  this->SetName("OBRRTStrategy");
};

template<class MPTraits>
OBRRTStrategy<MPTraits>::OBRRTStrategy(MPProblemType* _problem, XMLNodeReader& _node) : 
  BasicRRTStrategy<MPTraits>(_problem, _node, false), m_medialAxisUtility(_problem, _node){
    this->SetName("OBRRTStrategy");
    ParseXML(_node);
  };

template<class MPTraits>
void
OBRRTStrategy<MPTraits>::ParseXML(XMLNodeReader& _node) {
  m_g0 = _node.numberXMLParameter("g0", false, 0.1, 0.0, 1.0, "g0 Growth Method");
  m_g1 = _node.numberXMLParameter("g1", false, 0.0, 0.0, 1.0, "g1 Growth Method");
  m_g2 = _node.numberXMLParameter("g2", false, 0.0, 0.0, 1.0, "g2 Growth Method");
  m_g3 = _node.numberXMLParameter("g3", false, 0.0, 0.0, 1.0, "g3 Growth Method");
  m_g4 = _node.numberXMLParameter("g4", false, 0.0, 0.0, 1.0, "g4 Growth Method");
  m_g5 = _node.numberXMLParameter("g5", false, 0.0, 0.0, 1.0, "g5 Growth Method");
  m_g6 = _node.numberXMLParameter("g6", false, 0.0, 0.0, 1.0, "g6 Growth Method");
  m_g7 = _node.numberXMLParameter("g7", false, 0.0, 0.0, 1.0, "g7 Growth Method"); 
  m_g8 = _node.numberXMLParameter("g8", false, 0.0, 0.0, 1.0, "g8 Growth Method");

  _node.warnUnrequestedAttributes();

  //Normalize probabilities
  double total = m_g0 + m_g1 + m_g2 + m_g3 + m_g4 + m_g5 + m_g6 + m_g7 + m_g8;
  m_g0 = m_g0/total;
  m_g1 = m_g1/total;
  m_g2 = m_g2/total;
  m_g3 = m_g3/total;
  m_g4 = m_g4/total;
  m_g5 = m_g5/total;
  m_g6 = m_g6/total;
  m_g7 = m_g7/total;
  m_g8 = m_g8/total;

  m_g0N  = m_g0;
  m_g1N = m_g0N + m_g1;
  m_g2N = m_g1N + m_g2;
  m_g3N = m_g2N + m_g3;
  m_g4N = m_g3N + m_g4;
  m_g5N = m_g4N + m_g5;
  m_g6N = m_g5N + m_g6;
  m_g7N = m_g6N + m_g7;
  m_g8N = m_g7N + m_g8;
  if(this->m_debug) cout << " growth prob: ["
    << m_g0 << ","
      << m_g1 << ","
      << m_g2 << ","
      << m_g3 << ","
      << m_g4 << ","
      << m_g5 << ","
      << m_g6 << ","
      << m_g7 << ","
      << m_g8 << "]" << endl;
  if(this->m_debug) cout << " growth prob norms: ["
    << m_g0N << ","
      << m_g1N << ","
      << m_g2N << ","
      << m_g3N << ","
      << m_g4N << ","
      << m_g5N << ","
      << m_g6N << ","
      << m_g7N << ","
      << m_g8N << "]" << endl;
}

template<class MPTraits>
typename OBRRTStrategy<MPTraits>::VID
OBRRTStrategy<MPTraits>::ExpandTree(CfgType& _dir){
  if(this->m_debug) cout << " OBRRTStrategy::ExpandTree -- in expand call" << endl;
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  NeighborhoodFinderPointer nf = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nf);
  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  vector<pair<VID, double> > kClosest;
  vector<CfgType> cfgs;

  nf->FindNeighbors(this->GetMPProblem()->GetRoadmap(), _dir, back_inserter(kClosest));
  #ifndef _PARALLEL
  CfgType& nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0].first);
  #else
  CfgType nearest = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(kClosest[0].first);
  #endif
  CfgType newCfg;
  bool verifiedValid = false;

  // Next, find expansion method and apply
  // inside each growth method, the expansion is also checked
  double growthProb = DRand();
  if(growthProb < m_g0N) {
    /*if(this->m_debug)*/ cout << " calling g0: standard" << endl;
    newCfg = g0(nearest, _dir, verifiedValid);
  }
  else if(growthProb < m_g1N) {
    /*if(this->m_debug)*/ cout << " calling g1: random pos, same ori" << endl;
    newCfg = g1(nearest, _dir, verifiedValid);
  }
  else if(growthProb < m_g2N) {
    /*if(this->m_debug)*/ cout << " calling g2: rand obst. vec, rand ori" << endl;
    newCfg = g2(nearest, _dir, verifiedValid, false);
  }
  else if(growthProb < m_g3N) {
    /*if(this->m_debug)*/ cout << " calling g3: rand obst. vec, same ori" << endl;
    newCfg = g2(nearest, _dir, verifiedValid, true);
  }
  else if(growthProb < m_g4N) {
    /*if(this->m_debug)*/ cout << " calling g4: rotation, followed by extension" << endl;
    newCfg = g4(nearest, _dir, verifiedValid);
  }
  else if(growthProb < m_g5N) {
    /*if(this->m_debug)*/ cout << " calling g5: trace obstacle, random ori." << endl;
    newCfg = g5(nearest, _dir, verifiedValid, false);
  }
  else if(growthProb < m_g6N) {
    /*if(this->m_debug)*/ cout << " calling g6: trace obstacle, same" << endl;
    newCfg = g5(nearest, _dir, verifiedValid, true);
  }
  else if(growthProb < m_g7N) {
    /*if(this->m_debug)*/ cout << " calling g7: trace c-space obst" << endl;
    newCfg = g7(nearest, _dir, verifiedValid);
  }
  else { //g8
    /*if(this->m_debug)*/ cout << " calling g8" << endl;
    newCfg = g8(nearest, _dir, verifiedValid);
  }

  // If good to go, add to roadmap
  if(verifiedValid && dm->Distance(env, newCfg, nearest) >= this->m_minDist) {
    recentVID = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newCfg);
    cout << "Expanded tree to recent vid::" << recentVID << "::with parent::" << kClosest[0].first << endl;
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType(), WeightType());
    this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(kClosest[0].first, recentVID, weights);
  } 

  if(this->m_debug) cout << " OBRRTStrategy::ExpandTree -- done call" << endl;
  return recentVID;
}

/////////////////////////////////////////////////////////////////////////////////////////
//0.Standard RRT Expand
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g0(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo  cdInfo;
  CfgType newCfg;
  int weight;
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg;  
}

/////////////////////////////////////////////////////////////////////////////////////////
//1.Random position, same orientation
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g1(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo  cdInfo;
  CfgType newCfg;

  for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
    _dir[i] = _near[i];
  }
  int weight;
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//2&3. Obstacle Vector, random orientation if _maintainSrcOri=false, otherwise same orientation
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g2(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo  cdInfo;
  CfgType newCfg;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  //get an obstacle vector from env
  int numBodies = env->GetUsableMultiBodyCount();
  if( numBodies > 1 ) {//this growth method only works with obstacles (need 2 multibodies)

    int randIndex = (LRand() % (numBodies-1)) + 1;
    GMSPolyhedron& poly = env->GetMultiBody(randIndex)->GetFixedBody(0)->GetWorldPolyhedron();
    vector<Vector3d>& vertexList    = poly.m_vertexList; 
    vector<GMSPolygon>& polygonList = poly.m_polygonList; 
    //random polygon
    int randPolyInd = LRand() % polygonList.size();
    int randEdgeInd = LRand() % 3;
    int v1Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
    randEdgeInd = (randEdgeInd+1)%3;
    int v2Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
    Vector3d vertex1 = vertexList[v1Ind];
    Vector3d vertex2 = vertexList[v2Ind];
    Vector3d OV = vecScale * ( vertex1 - vertex2); //make obstacle vector
    if( DRand() < 0.5 ) OV = -1.0 * OV; //half the time switch direction

    //apply this obstacle vector
    for(size_t i = 0; i < _dir.PosDOF(); i++){
      _dir[i] = _near[i] + OV[i];
    }
    if( _maintainSrcOri ) {
      for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
        _dir[i] = _near[i];
      }
    }
    int weight;
    if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
      if(this->m_debug) cout << "RRT could not expand!" << endl; 
    }
    else {
      _verifiedValid = true;
    }
  }

  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//4. rotation followed by extension
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g4(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo  cdInfo;
  CfgType newCfg1, newCfg2;
  CfgType dirOrig = _dir;

  // setup cfg1 rotation component
  for(size_t i = 0; i < _dir.PosDOF(); i++){
    _dir[i] = _near[i];
  }
  int weight;
  // rotation first 
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg1, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
    // go from newCfg to newCfg2
    CfgType newNear = newCfg1;
    CfgType newDir  = newCfg1;
    // setup cfg2 translational component
    for(size_t i = 0; i < newDir.PosDOF(); i++){
      newDir[i] = dirOrig[i];
    }
    if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, newNear, newDir, newCfg2, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
      if(this->m_debug) cout << "RRT could not expand!" << endl; 
    }
    else{
      CfgType col;
      LPOutput<MPTraits> lpOutput;
      if(this->GetMPProblem()->GetLocalPlanner(this->m_lp)->IsConnected(env, *this->GetMPProblem()->GetStatClass(), 
            this->GetMPProblem()->GetDistanceMetric(this->m_dm), _near, newCfg2, col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes())) 
        return newCfg2;
    }
  }

  return newCfg1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//5&6. Trace Obstacle, random orientation if _maintainSrcOri=false, otherwise same orientation
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g5(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo cdInfo;
  CfgType newCfg;

  int weight;
  // rotation first 
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }

  //cout << " expand succeeded...check out cdInfo. "<<endl;
  //cout << "-- obst index: " << cdInfo.m_collidingObstIndex <<endl;
  //cout << "-- rapid contact1: " << cdInfo.m_rapidContactID1 <<endl;
  //cout << "-- rapid contact2: " << cdInfo.m_rapidContactID2 <<endl;

  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  //this kind of assumes cdInfo has values set which currently only RAPID does
  //correctly 
  int cIndex = cdInfo.m_collidingObstIndex;
  int obsContactIndex = cdInfo.m_rapidContactID2; 
  int numBodies = env->GetUsableMultiBodyCount();

  if( cIndex == -1 || obsContactIndex == -1 ) {
    if( cIndex == -1 ) { //nothing was set right, just do it randomly
      cIndex = (LRand() % (numBodies-1)) + 1;
      obsContactIndex = -1;
    }
  }
  GMSPolyhedron& poly = env->GetMultiBody(cIndex)->GetFixedBody(0)->GetWorldPolyhedron();
  vector<Vector3d>& vertexList    = poly.m_vertexList; 
  vector<GMSPolygon>& polygonList = poly.m_polygonList; 
  //random polygon
  int randPolyInd = LRand() % polygonList.size();
  if( obsContactIndex != -1 ) randPolyInd = obsContactIndex;
  int randEdgeInd = LRand() % 3;
  int v1Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
  randEdgeInd = (randEdgeInd+1)%3;
  int v2Ind= polygonList[randPolyInd].m_vertexList[randEdgeInd];
  Vector3d vertex1 = vertexList[v1Ind];
  Vector3d vertex2 = vertexList[v2Ind];
  Vector3d OV = vecScale * ( vertex1 - vertex2); //make obstacle vector
  if( DRand() < 0.5 ) OV = -1.0 * OV; //half the time switch direction

  //apply this obstacle vector
  for(size_t i = 0; i < _dir.PosDOF(); i++){
    _dir[i] = _near[i] + OV[i];
  }
  if( _maintainSrcOri ) {
    for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
      _dir[i] = _near[i];
    }
  }

  // rotation first 
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }


  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//7. C-space Obstacle
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g7(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(this->m_dm);
  CDInfo cdInfo;
  CfgType newCfg1, newCfg2, newCfg3;
  CfgType dir1 = _dir;
  CfgType dir2 = _dir;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vscale = 1;
  dir2.GetRandomRay(vscale, env, dm);

  int weight;
  // expand to c1  
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, dir1, newCfg1, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }

  // expand to c2 
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, dir2, newCfg2, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }

  // subtract newCfg2 and newCfg1 to get cspace vec
  CfgType cspaceDir;
  if(DRand() < 0.5)
    cspaceDir = newCfg2 - newCfg1;
  else
    cspaceDir = newCfg1 - newCfg2;

  _dir = _near + cspaceDir;

  dm->ScaleCfg(env, vscale, _near, _dir, true);

  // final expand
  if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, _dir, newCfg3, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
    if(this->m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg3;
}

/////////////////////////////////////////////////////////////////////////////////////////
//8. Medial Axis push
template<class MPTraits>
typename MPTraits::CfgType
OBRRTStrategy<MPTraits>::g8(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = this->GetMPProblem()->GetEnvironment();
  CDInfo cdInfo;
  CfgType newCfg, newCfg2;
  bool vv = false;

  CfgType tNear(_near);
  CfgType tDir(_dir);
  newCfg = g5(tNear, tDir, vv, false);


  if(!m_medialAxisUtility.PushToMedialAxis(newCfg, this->GetMPProblem()->GetEnvironment()->GetBoundary())){
    return newCfg;// CfgType(); //Error out   
  }
  else{ //pushed to medial axis, now check RRTexpand
    int weight;
    if(!RRTExpand<MPTraits>(this->GetMPProblem(), this->m_vc, this->m_dm, _near, newCfg, newCfg2, this->m_delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes())) {
      if(this->m_debug) cout << "RRT could not expand!" << endl; 
    }
    else {
      _verifiedValid = true;
    }
    return newCfg2;     
  }
}

#endif
