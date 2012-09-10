#include "OBRRTStrategy.h"
#include "CollisionDetection.h"
#include "DistanceMetrics.h"

void
OBRRTStrategy::ParseXML(XMLNodeReader& _node) {
  m_g0 = _node.numberXMLParameter("g0", false, 0.1, 0.0, 1.0, "g0 Growth Method");
  m_g1 = _node.numberXMLParameter("g1", false, 0.0, 0.0, 1.0, "g1 Growth Method");
  m_g2 = _node.numberXMLParameter("g2", false, 0.0, 0.0, 1.0, "g2 Growth Method");
  m_g3 = _node.numberXMLParameter("g3", false, 0.0, 0.0, 1.0, "g3 Growth Method");
  m_g4 = _node.numberXMLParameter("g4", false, 0.0, 0.0, 1.0, "g4 Growth Method");
  m_g5 = _node.numberXMLParameter("g5", false, 0.0, 0.0, 1.0, "g5 Growth Method");
  m_g6 = _node.numberXMLParameter("g6", false, 0.0, 0.0, 1.0, "g6 Growth Method");
  m_g7 = _node.numberXMLParameter("g7", false, 0.0, 0.0, 1.0, "g7 Growth Method"); 
  m_g8 = _node.numberXMLParameter("g8", false, 0.0, 0.0, 1.0, "g8 Growth Method");

  //MAPRM values
  m_exact = _node.boolXMLParameter("exact", false, "", "Exact Medial Axis Calculation");
  m_rayCount = _node.numberXMLParameter("rays", false, 20, 0, 50, "Number of Clearance Rays");
  m_penetration = _node.numberXMLParameter("penetration", false, 5, 0, 50, "Pentration");
  m_useBbx = _node.boolXMLParameter("useBBX", true, "", "Use Bounding Box");
  m_hLen = _node.numberXMLParameter("hLen", false, 5, 0, 20, "History Length");
  m_positional = _node.boolXMLParameter("positional", true, "", "Use Position in MA Calculations");
  m_debug = _node.boolXMLParameter("debug", false, "", "Debug Mode");

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
  cout << " growth prob norms: ["
    << m_g0 << ","
    << m_g1 << ","
    << m_g2 << ","
    << m_g3 << ","
    << m_g4 << ","
    << m_g5 << ","
    << m_g6 << ","
    << m_g7 << ","
    << m_g8 << "]" << endl;
  cout << " growth prob norms: ["
    << m_g0N << ","
    << m_g1N << ","
    << m_g2N << ","
    << m_g3N << ","
    << m_g4N << ","
    << m_g5N << ","
    << m_g6N << ","
    << m_g7N << ","
    << m_g8N << "]" << endl;

  if(m_debug) PrintOptions(cout);
}

OBRRTStrategy::VID
OBRRTStrategy::ExpandTree(CfgType& _dir){
  cout << " OBRRTStrategy::ExpandTree -- in expand call" << endl;
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  NeighborhoodFinder* nf = GetMPProblem()->GetNeighborhoodFinder();
  VID recentVID = INVALID_VID;
  // Find closest Cfg in map
  vector<VID> kClosest;
  vector<CfgType> cfgs;

  nf->GetMethod(m_nf)->KClosest(GetMPProblem()->GetRoadmap(), _dir, 1, back_inserter(kClosest));     
  CfgType nearest = GetMPProblem()->GetRoadmap()->m_pRoadmap->find_vertex(kClosest[0])->property();
  CfgType newCfg;
  bool verifiedValid = false;

  // Next, find expansion method and apply
  // inside each growth method, the expansion is also checked
  double growth_prob = DRand();
  if( growth_prob < m_g0N ) {
    cout << " calling g0: standard" << endl;
    newCfg = g0( nearest, _dir, verifiedValid );
  }
  else if( growth_prob < m_g1N ) {
    cout << " calling g1: random pos, same ori" << endl;
    newCfg = g1( nearest, _dir, verifiedValid );
  }
  else if( growth_prob < m_g2N ) {
    cout << " calling g2: rand obst. vec, rand ori" << endl;
    newCfg = g2( nearest, _dir, verifiedValid );
  }
  else if( growth_prob < m_g3N ) {
    cout << " calling g3: rand obst. vec, same ori" << endl;
    newCfg = g3( nearest, _dir, verifiedValid );
  }
  else if( growth_prob < m_g4N ) {
    cout << " calling g4: rotation, followed by extension" << endl;
    newCfg = g4( nearest, _dir, verifiedValid );
  }
  else if( growth_prob < m_g5N ) {
    cout << " calling g5: trace obstacle, random ori." << endl;
    newCfg = g5( nearest, _dir, verifiedValid, false );
  }
  else if( growth_prob < m_g6N ) {
    cout << " calling g6: trace obstacle, same" << endl;
    newCfg = g5( nearest, _dir, verifiedValid, true );
  }
  else if( growth_prob < m_g7N ) {
    cout << " calling g7: trace c-space obst" << endl;
    newCfg = g7( nearest, _dir, verifiedValid );
  }
  else { //g8
    cout << " calling g8" << endl;
    newCfg = g8( nearest, _dir, verifiedValid );
  }

  // If good to go, add to roadmap
  if(verifiedValid && dm->Distance(env, newCfg, nearest) >= m_minDist) {
    recentVID = GetMPProblem()->GetRoadmap()->m_pRoadmap->AddVertex(newCfg);
    //TODO fix weight
    pair<WeightType, WeightType> weights = make_pair(WeightType(), WeightType());
    GetMPProblem()->GetRoadmap()->m_pRoadmap->AddEdge(nearest, newCfg, weights);
  } 

  cout << " OBRRTStrategy::ExpandTree -- done call" << endl;
  return recentVID;
}

/////////////////////////////////////////////////////////////////////////////////////////
//0.Standard RRT Expand
CfgType OBRRTStrategy::g0(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo  cdInfo;
  CfgType newCfg;
  int weight;
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg;  
}

/////////////////////////////////////////////////////////////////////////////////////////
//1.Random position, same orientation
CfgType OBRRTStrategy::g1(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo  cdInfo;
  CfgType newCfg;

  for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
    _dir.SetSingleParam(i, _near.GetData()[i]);
  }
  int weight;
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//2. Obstacle Vector, random orientation
CfgType OBRRTStrategy::g2(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo  cdInfo;
  CfgType newCfg;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  //get an obstacle vector from env
  int numBodies = env->GetMultiBodyCount();
  if( numBodies > 1 ) {//this growth method only works with obstacles (need 2 multibodies)

    int randIndex = (LRand() % (numBodies-1)) + 1;
    GMSPolyhedron& poly = env->GetMultiBody(randIndex)->GetFixedBody(0)->GetWorldPolyhedron();
    vector<Vector3D>& m_vertexList    = poly.m_vertexList; 
    vector<GMSPolygon>& m_polygonList = poly.m_polygonList; 
    //random polygon
    int randPolyInd = LRand() % m_polygonList.size();
    int randEdgeInd = LRand() % 3;
    int v1Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
    randEdgeInd = (randEdgeInd+1)%3;
    int v2Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
    Vector3D vertex1 = m_vertexList[v1Ind];
    Vector3D vertex2 = m_vertexList[v2Ind];
    Vector3D OV = vecScale * ( vertex1 - vertex2); //make obstacle vector
    if( DRand() < 0.5 ) OV = -1.0 * OV; //half the time switch direction

    //apply this obstacle vector
    for(size_t i = 0; i < _dir.PosDOF(); i++){
      _dir.SetSingleParam(i, _near.GetSingleParam(i) + OV[i]);
    }
    int weight;
    if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
      if(m_debug) cout << "RRT could not expand!" << endl; 
    }
    else {
      _verifiedValid = true;
    }
  }

  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//3. Obstacle Vector, same orientation
CfgType OBRRTStrategy::g3(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo  cdInfo;
  CfgType newCfg;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  //get an obstacle vector from env
  int numBodies = env->GetMultiBodyCount();
  if( numBodies > 1 ) {//this growth method only works with obstacles (need 2 multibodies)

    int randIndex = (LRand() % (numBodies-1)) + 1;
    GMSPolyhedron& poly = env->GetMultiBody(randIndex)->GetFixedBody(0)->GetWorldPolyhedron();
    vector<Vector3D>& m_vertexList    = poly.m_vertexList; 
    vector<GMSPolygon>& m_polygonList = poly.m_polygonList; 
    //random polygon
    int randPolyInd = LRand() % m_polygonList.size();
    int randEdgeInd = LRand() % 3;
    int v1Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
    randEdgeInd = (randEdgeInd+1)%3;
    int v2Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
    Vector3D vertex1 = m_vertexList[v1Ind];
    Vector3D vertex2 = m_vertexList[v2Ind];
    Vector3D OV = vecScale * ( vertex1 - vertex2); //make obstacle vector
    if( DRand() < 0.5 ) OV = -1.0 * OV; //half the time switch direction

    //apply this obstacle vector
    for(size_t i = 0; i < _dir.PosDOF(); i++){
      _dir.SetSingleParam(i, _near.GetSingleParam(i) + OV[i]);
    }
    for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
      _dir.SetSingleParam(i, _near.GetSingleParam(i) );
    }
    int weight;
    if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
      if(m_debug) cout << "RRT could not expand!" << endl; 
    }
    else {
      _verifiedValid = true;
    }
  }

  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//4. rotation followed by extension
CfgType OBRRTStrategy::g4(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo  cdInfo;
  CfgType newCfg1, newCfg2;
  CfgType dirOrig = _dir;

  // setup cfg1 rotation component
  for(size_t i = 0; i < _dir.PosDOF(); i++){
    _dir.SetSingleParam(i, _near.GetSingleParam(i) );
  }
  int weight;
  // rotation first 
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg1, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
    // go from newCfg to newCfg2
    CfgType newNear = newCfg1;
    CfgType newDir  = newCfg1;
    // setup cfg2 translational component
    for(size_t i = 0; i < newDir.PosDOF(); i++){
      newDir.SetSingleParam(i, dirOrig.GetSingleParam(i) );
    }
    if(!RRTExpand(GetMPProblem(), m_vc, m_dm, newNear, newDir, newCfg2, m_delta, weight, cdInfo)) {
      if(m_debug) cout << "RRT could not expand!" << endl; 
    }
    else return newCfg2;
  }

  return newCfg1;
}

/////////////////////////////////////////////////////////////////////////////////////////
//5&6. Trace Obstacle, rand ori
CfgType OBRRTStrategy::g5(CfgType& _near, CfgType& _dir, bool& _verifiedValid, bool _maintainSrcOri){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo cdInfo;
  CfgType newCfg;

  int weight;
  // rotation first 
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }

  //cout << " expand succeeded...check out cdInfo. "<<endl;
  //cout << "-- obst index: " << cdInfo.colliding_obst_index <<endl;
  //cout << "-- rapid contact1: " << cdInfo.rapid_contact_id1 <<endl;
  //cout << "-- rapid contact2: " << cdInfo.rapid_contact_id2 <<endl;

  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vecScale = 10.0;

  //this kind of assumes cdInfo has values set which currently only RAPID does
  //correctly 
  int cIndex = cdInfo.colliding_obst_index;
  int obsContactIndex = cdInfo.rapid_contact_id2; 
  int numBodies = env->GetMultiBodyCount();

  if( cIndex == -1 || obsContactIndex == -1 ) {
    if( cIndex == -1 ) { //nothing was set right, just do it randomly
      cIndex = (LRand() % (numBodies-1)) + 1;
      obsContactIndex = -1;
    }
  }
  GMSPolyhedron& poly = env->GetMultiBody(cIndex)->GetFixedBody(0)->GetWorldPolyhedron();
  vector<Vector3D>& m_vertexList    = poly.m_vertexList; 
  vector<GMSPolygon>& m_polygonList = poly.m_polygonList; 
  //random polygon
  int randPolyInd = LRand() % m_polygonList.size();
  if( obsContactIndex != -1 ) randPolyInd = obsContactIndex;
  int randEdgeInd = LRand() % 3;
  int v1Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
  randEdgeInd = (randEdgeInd+1)%3;
  int v2Ind= m_polygonList[randPolyInd].m_vertexList[randEdgeInd];
  Vector3D vertex1 = m_vertexList[v1Ind];
  Vector3D vertex2 = m_vertexList[v2Ind];
  Vector3D OV = vecScale * ( vertex1 - vertex2); //make obstacle vector
  if( DRand() < 0.5 ) OV = -1.0 * OV; //half the time switch direction

  //apply this obstacle vector
  for(size_t i = 0; i < _dir.PosDOF(); i++){
    _dir.SetSingleParam(i, _near.GetSingleParam(i) + OV[i]);
  }
  if( _maintainSrcOri ) {
    for(size_t i = _dir.PosDOF(); i < _dir.DOF(); i++){
      _dir.SetSingleParam(i, _near.GetSingleParam(i));
    }
  }

  // rotation first 
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }


  return newCfg;
}

/////////////////////////////////////////////////////////////////////////////////////////
//7. C-space Obstacle
CfgType OBRRTStrategy::g7(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  CDInfo cdInfo;
  CfgType newCfg1, newCfg2, newCfg3;
  CfgType dir1 = _dir;
  CfgType dir2 = _dir;
  //VECTOR SCALE - THIS WILL BE HARD CODED BUT SHOULD PROBABLY BE MADE AN OPTION
  double vscale = 1;
  dir2.GetRandomRay(vscale, env, dm);
  //dir2.GetRandomCfg( vscale, 1.0 );
  //dir2.GetRandomCfg( env );

  int weight;
  // expand to c1  
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, dir1, newCfg1, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }

  // expand to c2 
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, dir2, newCfg2, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }

  // subtract newCfg2 and newCfg1 to get cspace vec
  CfgType cspaceDir;
  if( DRand() < 0.5 ) cspaceDir.subtract( newCfg2, newCfg1 );
  else                cspaceDir.subtract( newCfg1, newCfg2 );

  for(size_t i = 0; i < _dir.DOF(); i++){
    _dir.SetSingleParam(i, _near.GetSingleParam(i) + cspaceDir.GetSingleParam(i));
  }

  dm->ScaleCfg(env, vscale, _near, _dir, true);

  // final expand
  if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, _dir, newCfg3, m_delta, weight, cdInfo)) {
    if(m_debug) cout << "RRT could not expand!" << endl; 
  }
  else {
    _verifiedValid = true;
  }

  return newCfg3;
}

/////////////////////////////////////////////////////////////////////////////////////////
//8. Medial Axis push
CfgType OBRRTStrategy::g8(CfgType& _near, CfgType& _dir, bool& _verifiedValid){
  // Setup MP Variables
  Environment* env = GetMPProblem()->GetEnvironment();
  shared_ptr<DistanceMetricMethod> dm = GetMPProblem()->GetDistanceMetric()->GetMethod(m_dm);
  StatClass* stats = GetMPProblem()->GetStatClass();
  CDInfo cdInfo;
  CfgType newCfg, newCfg2;
  bool vv = false;

  CfgType tNear(_near);
  CfgType tDir(_dir);
  newCfg = g5( tNear, tDir, vv, false );


  if(!PushToMedialAxis(GetMPProblem(), env, newCfg, *stats, m_vc, m_dm, m_exact, m_rayCount, m_exact, m_penetration, m_useBbx, .0001, m_hLen, m_debug, m_positional)){
    return newCfg;// CfgType(); //Error out   
  }
  else{ //pushed to medial axis, now check RRTexpand
    int weight;
    if(!RRTExpand(GetMPProblem(), m_vc, m_dm, _near, newCfg, newCfg2, m_delta, weight, cdInfo)) {
      if(m_debug) cout << "RRT could not expand!" << endl; 
    }
    else {
      _verifiedValid = true;
    }
    return newCfg2;     
  }
}



