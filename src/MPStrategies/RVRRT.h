#ifndef RVRRTSTRATEGY_H_
#define RVRRTSTRATEGY_H_
#include "ReachableVolumeUtil/ReachableVolumeRobot.h"
#include "BasicRRTStrategy.h"
//#include <boost/shared_ptr.hpp>
#include "DistanceMetrics/RVDistance.h"
//using namespace boost;
//using shared_ptr;

typedef Vector3d Vector3D;


ReachableVolumeRobot g_reachableVolumeRobotsRVRRT;
bool init=false;

// note, we may note need node sample if stored in rvr
////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
class PerturbJoints{
 public:
  shared_ptr<vector<Vector3D> > operator()(ReachableVolumeRobot &_rvr,  vector<Vector3D> *_node, vector<Vector3D> *_perturbTowards, double _delta, string _perturbDir = "WorkspaceDir", string _repositionPolicy = "Random"){
    shared_ptr<vector<Vector3D> > returnValue;
    return returnValue;
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
//Randomly select a joint from _node and perturb it by epsilon in directorn of that joint in _perturbTowards
class PerturbOneRandomJoint : public PerturbJoints{
 public:

  vector<Vector3D> *operator()(ReachableVolumeRobot &_rvr,  vector<Vector3D> *_node, vector<Vector3D> *_perturbTowards, double _delta, string _perturbDir = "WorkspaceDir", string _repositionPolicy = "Random"){
    //Find rvj for node from rvr
    //perturb node and check if still in rv
    //recursivly resample if node not in rv
    //return sample

    vector<Vector3D> *newNode= new vector<Vector3D>(_node->begin(),_node->end());

    for(int i=0; i<=_rvr.m_nAttempts; i++){
      int jointId = 1 + rand() % (_node->size()-2);
      Vector3D perturbTowards;
      if(_perturbDir.compare("WorkspaceDir")==0){
	perturbTowards=(*_perturbTowards)[jointId];
      }else if(_perturbDir.compare("RVSpaceDir")==0){
        perturbTowards=(*_perturbTowards)[jointId]-(*_perturbTowards)[0]+(*_node)[0];  //position of joint if it were in rv space of _node
      }else{
        perturbTowards=(*_perturbTowards)[jointId];
      }

      if(_rvr.perturbJoint(*newNode, jointId, (*_perturbTowards)[jointId],_delta,_repositionPolicy)){
	return newNode;
      }
    }

    delete(newNode);
    return new vector<Vector3D>;
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
//Randomly select a joint that is closest
class PerturbClosestJoint : public PerturbJoints{
 public:

  vector<Vector3D> *operator()(ReachableVolumeRobot &_rvr,  vector<Vector3D> *_node, vector<Vector3D> *_perturbTowards, double _delta, string _perturbDir = "WorkspaceDir", string _repositionPolicy = "Random"){
    //Find rvj for node from rvr
    //perturb node and check if still in rv
    //recursivly resample if node not in rv
    //return sample

    vector<Vector3D> *newNode = new vector<Vector3D>(_node->begin(),_node->end());
    int jointId=1;
    double d = ReachableVolume::distance((*_perturbTowards)[0],(*_node)[0]);
    for(unsigned int j=1; j<newNode->size(); j++){
      double d2=ReachableVolume::distance((*_perturbTowards)[j],(*_node)[j]);
      if(d2<d){
	d=d2;
	jointId=j;
      }
    }

    Vector3D perturbTowards;
    if(_perturbDir.compare("WorkspaceDir")==0){
      perturbTowards=(*_perturbTowards)[jointId];
    }else if(_perturbDir.compare("RVSpaceDir")==0){
      perturbTowards=(*_perturbTowards)[jointId]-(*_perturbTowards)[0]+(*_node)[0];  //position of joint if it were in rv space of _node
    }else{
      perturbTowards=(*_perturbTowards)[jointId];
    }

    if(_rvr.perturbJoint(*newNode, jointId, (*_perturbTowards)[jointId],_delta,_repositionPolicy)){
      return newNode;
    }

    delete(newNode);
    return new vector<Vector3D>;
  }
};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategyUtils
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
//Selects most distant joint
class PerturbMostDistantJoint : public PerturbJoints{
 public:

  vector<Vector3D> *operator()(ReachableVolumeRobot &_rvr,  vector<Vector3D> *_node, vector<Vector3D> *_perturbTowards, double _delta, string _perturbDir = "WorkspaceDir", string _repositionPolicy = "Random"){
    //Find rvj for node from rvr
    //perturb node and check if still in rv
    //recursivly resample if node not in rv
    //return sample

    vector<Vector3D> *newNode = new vector<Vector3D>(_node->begin(),_node->end());
    int jointId=1;
    double d = ReachableVolume::distance((*_perturbTowards)[0],(*_node)[0]);
    for(unsigned int j=1; j<newNode->size(); j++){
      double d2=ReachableVolume::distance((*_perturbTowards)[j],(*_node)[j]);
      if(d2>d){
        d=d2;
        jointId=j;
      }
    }

    Vector3D perturbTowards;
    if(_perturbDir.compare("WorkspaceDir")==0){
      perturbTowards=(*_perturbTowards)[jointId];
    }else if(_perturbDir.compare("RVSpaceDir")==0){
      perturbTowards=(*_perturbTowards)[jointId]-(*_perturbTowards)[0]+(*_node)[0];  //position of joint if it were in rv space of _node
    }else{
      perturbTowards=(*_perturbTowards)[jointId];
    }

    if(_rvr.perturbJoint(*newNode, jointId, (*_perturbTowards)[jointId],_delta,_repositionPolicy)){
      return newNode;
    }

    delete(newNode);
    return new vector<Vector3D>;

  }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningStrategies
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
///
/// \todo Configure for pausible execution.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class ReachableVolumeRRT : public BasicRRTStrategy<MPTraits> {
 public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    map<VID, shared_ptr<vector<Vector3D> > > m_rvCfgs;  //whenever adding node also add to this, later we may want to add this to cfg instead
    double m_deltaRV;
    double m_S;
    double m_S_rot;

    string m_treeStructure;
    string m_jointPerturbMethod;
    string m_perturbDir;
    string m_repositionPolicy;



  virtual void Run();

  ReachableVolumeRRT():BasicRRTStrategy<MPTraits>(){
    if(!init){
      g_reachableVolumeRobotsRVRRT.loadTree("Chain.tree",m_treeStructure);
      init=true;
    }
    this->SetName("ReachableVolumeRRT");
  }

  ReachableVolumeRRT(string _lp, string _dm,
		     string _nf, string _vc, string _nc, string _gt, vector<string> _evaluators,
		     double _delta, double _minDist, double _growthFocus, bool _evaluateGoal, CfgType _start, CfgType _goal,
		     size_t _numRoots, size_t _numDirections, bool _growGoals,char* _chainFile = "Chain.tree"):BasicRRTStrategy<MPTraits>(_lp,_dm,_nf,_vc,_nc,_gt,_evaluators,_delta,_minDist,_growthFocus,_evaluateGoal,_start,_goal,_numRoots,_numDirections,_growGoals){//passing everything but the kitchen sink...
    //cout<<"loading 2"<<m_treeStructure<<endl;
    if(!init){
      g_reachableVolumeRobotsRVRRT.loadTree(_chainFile,m_treeStructure);
      init=true;
    }
    this->SetName("ReachableVolumeRRT");
  }


  ReachableVolumeRRT(MPProblemType* _problem, XMLNode& _node, bool _warnXML=true):BasicRRTStrategy<MPTraits>(_problem, _node, false){
    ParseXMLRVRRT(_node);
  }

  void ParseXMLRVRRT(XMLNode& _node){
    this->m_deltaRV = _node.Read("deltaRV", false, .1, (double)0, (double)1000, "RV space delta value");
    this->m_S = _node.Read("s", false, .1, (double)0, (double)1, "Scaling Factor");
    this->m_S_rot = _node.Read("s_rot", false, .1, (double)0, (double)1, "Rotational Scaling Factor");
    this->m_treeStructure = _node.Read("treeStructure", false, "EndEffectorFirst", "Structure of reachable volume tree");
    this->m_perturbDir = _node.Read("perturbDir", false, "WorkspaceDir", "Policy for selecting direction of perturbation");
    this->m_repositionPolicy = _node.Read("repositionPolicy", false, "Random", "Policy for repositioning children that are outside of reachable volume");
    this->m_jointPerturbMethod = _node.Read("jointPerturbMethod", false, "OneRandomJoint", "Method for perturbing joints");
    if(!init){
      g_reachableVolumeRobotsRVRRT.loadTree(_node.Read("chainFile", false, "Chain.tree", "Chain file name").c_str(),this->m_treeStructure);
      init=true;
    }
  }

~ReachableVolumeRRT(){ }

shared_ptr<vector<Vector3D> > convertToJointPositions(CfgType _cfg){
  shared_ptr<vector<Vector3D> > joints = shared_ptr<vector<Vector3D> >(new vector<Vector3D>);
  _cfg.ConfigureRobot();
  this->GetEnvironment()->GetRobot(_cfg.GetRobotIndex())->PolygonalApproximation(*joints);
  for(unsigned int i=1; i<joints->size();i++){
    (*joints)[i]=(*joints)[i]-(*joints)[0];
  }
  //(*joints)[0]=(0,0,0);
  Vector3d origin; //(0,0,0)
  (*joints)[0]=origin;
  return joints;
}



void PrintOptions(ostream& _os) {
  typedef vector<string>::iterator SIT;
  _os << "ReachableVolumeRRT::PrintOptions" << endl;
  /*
  _os << "\tNeighborhood Finder:: " << this->m_nf << endl;
  _os << "\tDistance Metric:: " << this->m_dm << endl;
  _os << "\tValidity Checker:: " << this->m_vc << endl;
  _os << "\tLocal Planner:: " << m_lp << endl;
  _os << "\tConnection Method:: " << m_nc << endl;
  _os << "\tGraph Type:: " << m_gt << endl;
  _os << "\tEvaluate Goal:: " << m_evaluateGoal << endl;
  _os << "\tEvaluators:: " << endl;
  _os << "\tGrow Goals:: " << m_growGoals << endl;
  for(const auto& label : m_meLabels)
    _os << "\t\t" << label << endl;
  _os << "\tdelta:: " << m_delta << endl;
  _os << "\tminimum distance:: " << m_minDist << endl;
  _os << "\tnumber of roots:: " << m_numRoots << endl;
  _os << "\tgrowth focus:: " << m_growthFocus << endl;
  _os << "\tnumber of expansion directions:: " << m_numDirections << endl;
  */
}

    template<typename EnableCfg>
      EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2) {
      return _c1 - _c2;
    }

/*
    template<typename EnableCfg>
      EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
                              typename disable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      return _c1 - _c2;
    }
*/
};

template<class MPTraits>
void
ReachableVolumeRRT<MPTraits>::Run() {

  Environment* env = this->GetMPProblem()->GetEnvironment();
  NeighborhoodFinderPointer nfp = this->GetMPProblem()->
      GetNeighborhoodFinder(this->m_nfLabel);

  this->m_debug=false;
  if(this->m_debug) cout << "\nRunning RVRRTStrategy::" << endl;

  // Setup MP Variables

  StatClass* stats = this->GetMPProblem()->GetStatClass();

  stats->StartClock("RRT Generation");
  DistanceMetricPointer dm = this->GetMPProblem()->
      GetDistanceMetric(this->m_dmLabel);
  CfgType dir;

  bool mapPassedEvaluation = false;
  double minDist;
  typename GraphType::iterator nn;
  int nGoalsNotFound=this->m_trees.size();
  while(!mapPassedEvaluation){
    //get dir - need both rv cfg (for perturbation) and regular cfg (for dm)
    dir.GetRandomCfg(this->GetMPProblem()->GetEnvironment());
    if(this->m_debug) cout<<"dir = "<<dir<<endl;
    VID nnVid;
    CfgType nn;
    minDist=-1;
    //find nearest neibhbor to ran
    for(typename GraphType::VI i = this->GetMPProblem()->GetRoadmap()->GetGraph()->begin();  i!=this->GetMPProblem()->GetRoadmap()->GetGraph()->end(); i++){
      CfgType current = this->GetMPProblem()->GetRoadmap()->GetGraph()->GetVertex(i);
      double dist = dm->Distance(current,dir);
      if(minDist==-1 || minDist>dist){
	minDist=dist;
	nn=current;
	nnVid = (*i).descriptor();
      }

    }
    shared_ptr<vector<Vector3D> > dirJoints = convertToJointPositions(dir);
    typename map<VID, shared_ptr<vector<Vector3D> > >::iterator nnIter=m_rvCfgs.find(nnVid);
    shared_ptr<vector<Vector3D> > nnRvSample;
    if(nnIter!=m_rvCfgs.end()){
	nnRvSample=nnIter->second;
    }else{
      nnRvSample=convertToJointPositions(nn);
      m_rvCfgs[nnVid]=nnRvSample;
    }

    vector<Vector3D> tmp = *nnRvSample;

    vector<Vector3D> *newNode;
    double deltaRV = m_deltaRV;
    double deltaTrans = 0;

    if(!g_reachableVolumeRobotsRVRRT.m_fixed){
      //set delta and deltaTR using m_S
      CfgType diff=DifferenceCfg(dir, nn);
      double distanceRV=RVDistance<MPTraits>::InternalDistance(env,dir,nn);
      double distanceRot=RVDistance<MPTraits>::RotationalDistance(diff, 2);
      double distanceTr=RVDistance<MPTraits>::PositionDistance(env, diff, 2, .5, true);
      double distanceTotal=m_S*(distanceRot+distanceTr)+(1-m_S)*distanceRV;
      deltaTrans=m_deltaRV*m_S*(distanceRot+distanceTr)/distanceTotal;
      deltaRV=m_deltaRV-deltaTrans;
    }
    double deltaRot = m_S_rot*deltaTrans;
    deltaTrans-=deltaRot;
    if(m_jointPerturbMethod.compare("OneRandomJoint")==0){
      PerturbOneRandomJoint pj;
      newNode = pj(g_reachableVolumeRobotsRVRRT, &tmp, dirJoints.get(), deltaRV,this->m_perturbDir,this->m_repositionPolicy);
    }else if(m_jointPerturbMethod.compare("ClosestJoint")==0){
      PerturbClosestJoint pj;
      newNode = pj(g_reachableVolumeRobotsRVRRT, &tmp, dirJoints.get(), deltaRV,this->m_perturbDir,this->m_repositionPolicy);
    }else if(m_jointPerturbMethod.compare("MostDistantJoint")==0){
      PerturbMostDistantJoint pj;
      newNode = pj(g_reachableVolumeRobotsRVRRT, &tmp, dirJoints.get(), deltaRV,this->m_perturbDir,this->m_repositionPolicy);
    }else{
      cout<<"jointPerturbMethod = "<<this->m_jointPerturbMethod<<".  This method does not exist.  Defaulting to PerturbOneRandom method."<<endl;
      PerturbOneRandomJoint pj;
      newNode = pj(g_reachableVolumeRobotsRVRRT, &tmp, dirJoints.get(), deltaRV,this->m_perturbDir, this->m_repositionPolicy);
    }
    if(newNode==NULL || newNode==0){
      if(this->m_debug)cout<<"null pointer"<<endl;
      continue;
    }
    if(newNode->size()==0){
      if(this->m_debug)cout<<"size = 0"<<endl;
      continue;
    }
    if(this->m_debug)cout<<newNode->size()<<endl;
    vector<double> *jointAngleCfg = g_reachableVolumeRobotsRVRRT.convertToJointAngleSample(*newNode);
    //deal with translational coordinates
    //push rotational + translational coordinates by delta in dir of random node
    vector<double> cfgData(6);

    if(g_reachableVolumeRobotsRVRRT.m_fixed){
      //fixed base, all nodes should have the same base pos
      cfgData[0]=nn.GetData()[0];
      cfgData[0]=nn.GetData()[1];
      cfgData[0]=nn.GetData()[2];
      cfgData[0]=nn.GetData()[3];
      cfgData[0]=nn.GetData()[4];
      cfgData[0]=nn.GetData()[5];
    }else{
      //compute translational coordinates that are delta_trans away from nn in direction of dir
      double d_trans = sqrt((dir.GetData()[0]-nn.GetData()[0])*(dir.GetData()[0]-nn.GetData()[0])
			    +(dir.GetData()[1]-nn.GetData()[1])*(dir.GetData()[1]-nn.GetData()[1])
			    +(dir.GetData()[2]-nn.GetData()[2])*(dir.GetData()[2]-nn.GetData()[2]));

      cfgData[0]=nn.GetData()[0] + deltaTrans*(dir.GetData()[0]-nn.GetData()[0])/d_trans;
      cfgData[1]=nn.GetData()[1] + deltaTrans*(dir.GetData()[1]-nn.GetData()[1])/d_trans;
      cfgData[2]=nn.GetData()[2] + deltaTrans*(dir.GetData()[2]-nn.GetData()[2])/d_trans;


      //compute rotational coordinates that are delta_trans away from nn in direction of dir
      double d_rot = sqrt((dir.GetData()[3]-nn.GetData()[3])*(dir.GetData()[3]-nn.GetData()[3])
                            +(dir.GetData()[4]-nn.GetData()[4])*(dir.GetData()[4]-nn.GetData()[4])
                            +(dir.GetData()[5]-nn.GetData()[5])*(dir.GetData()[5]-nn.GetData()[5]));

      /*
      cfgData[3]=nn.GetData()[3] + this->m_deltaRot*(dir.GetData()[3]-nn.GetData()[3])/d_rot;
      cfgData[4]=nn.GetData()[4] + this->m_deltaRot*(dir.GetData()[4]-nn.GetData()[4])/d_rot;
      cfgData[5]=nn.GetData()[5] + this->m_deltaRot*(dir.GetData()[5]-nn.GetData()[5])/d_rot;
      */
      cfgData[3]=0;
      cfgData[4]=ReachableVolumeLinkage::getPitch(*newNode)/(2*PI) - ReachableVolumeLinkage::getPitch(*nnRvSample)/(2*PI) + nn.GetData()[4] + deltaRot*(dir.GetData()[4]-nn.GetData()[4])/d_rot;


      cfgData[5]=ReachableVolumeLinkage::getYaw(*newNode)/(2*PI) - ReachableVolumeLinkage::getYaw(*nnRvSample)/(2*PI) + nn.GetData()[5] + deltaRot*(dir.GetData()[5]-nn.GetData()[5])/d_rot;

      while(cfgData[3]>1)
        cfgData[4]-=1;
      while(cfgData[4]<-1)
        cfgData[4]+=1;

      while(cfgData[5]>1)
        cfgData[5]-=1;
      while(cfgData[5]<-1)
        cfgData[5]+=1;

    }
    for(unsigned int i=0; i<jointAngleCfg->size(); i++){
      cfgData.push_back((*jointAngleCfg)[i]);
    }
    delete(jointAngleCfg);
    CfgType newNodeCfg;  //convert new node to cfg
    newNodeCfg.SetData(cfgData);
    LPOutput<MPTraits> lpOutput;
    if(this->m_debug) cout<<"new node ="<<newNodeCfg<<endl;
    bool success = this->GetMPProblem()->GetLocalPlanner(this->m_lp)->
      IsConnected(nn, newNodeCfg, &lpOutput, this->GetMPProblem()->GetEnvironment()->GetPositionRes(), this->GetMPProblem()->GetEnvironment()->GetOrientationRes(),true,false,false);  //changed dir to nn here
    if(success){
      VID newNodeVid = this->GetMPProblem()->GetRoadmap()->GetGraph()->AddVertex(newNodeCfg);
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(nnVid,newNodeVid,WeightType());
      this->GetMPProblem()->GetRoadmap()->GetGraph()->AddEdge(newNodeVid,nnVid,WeightType());
      m_rvCfgs[newNodeVid] = shared_ptr<vector<Vector3D> >(newNode);
      if(this->m_growGoals){
	this->ConnectTrees(newNodeVid);
      }else{
	for(vector<size_t>::iterator i = this->m_goalsNotFound.begin(); i!=this->m_goalsNotFound.end(); i++){
	  vector<pair<VID, double> > closest;
	  bool connectToGoal = this->GetMPProblem()->GetLocalPlanner(this->m_lp)->
	    IsConnected(nn, newNodeCfg, &lpOutput, this->GetMPProblem()->GetEnvironment()->GetPositionRes(), this->GetMPProblem()->GetEnvironment()->GetOrientationRes(),true,false,false);

	  if(connectToGoal){
	    this->m_goalsNotFound.erase(i);
            nGoalsNotFound--;
	    stats->StopClock("RRT Generation");
	    return;
          }
	}
      }
    }


    bool evalMap = this->EvaluateMap();

    if(!this->m_growGoals){
      if(nGoalsNotFound<=0){
	mapPassedEvaluation=true;
      }else{
	mapPassedEvaluation=evalMap;
      }
      if(this->m_goalsNotFound.size()==0)
        cout << "RRT FOUND ALL GOALS" << endl;
    }else{
      if(evalMap || this->m_trees.size()==1){
	mapPassedEvaluation=true;
      }
    }

    stats->StopClock("RRT Generation");
    if(this->m_debug) {
      stats->PrintClock("RRT Generation", cout);
      cout<<"\nEnd Running BasicRRTStrategy::" << endl;
    }

  }

}

#endif
