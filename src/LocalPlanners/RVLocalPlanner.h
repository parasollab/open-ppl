#ifndef RV_LOCAL_PLANNER_H_
#define RV_LOCAL_PLANNER_H_

#include <queue>
#include "LocalPlannerMethod.h"
#include "LPOutput.h"
#include "MPProblem/IsClosedChain.h"
#include "ReachableVolumeUtil/ReachableVolumeRobot.h"
#include "DistanceMetrics/RVDistance.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup LocalPlanners
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class RVLocalPlanner: public LocalPlannerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    ReachableVolumeRobot m_rvr;
    string m_treeStructure;
    string m_repositionPolicy;
    double m_rvres;
    double m_S;

    RVLocalPlanner(string _vcLabel = "", bool _evalation = false);
    RVLocalPlanner(MPProblemType* _problem, XMLNode& _node);
    virtual ~RVLocalPlanner();

    virtual void PrintOptions(ostream& _os);

    string GetName(){return "RVLocalPlanner";};

    /**
     * Check if two Cfgs can be connected.
     */

    virtual bool IsConnected(
        const CfgType& _c1, const CfgType& _c2, CfgType& _col,
        LPOutput<MPTraits>* _lpOutput, double _posRes, double _oriRes,
        bool _checkCollision = true, bool _savePath = false,
        bool _saveFailedPath = false){

      Environment* env = this->GetMPProblem()->GetEnvironment();
      ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      string callee = this->GetName() + "::IsConnectedRV";
      CDInfo cdInfo;
      CfgType tick;
      int nTicks=0;
      int tick_num=0;

      tick = _c1;
      vector<double> incriment;
      if(!m_rvr.m_fixed){
	CfgType diff=DifferenceCfg(_c1, _c2);
	double d_pos=RVDistance<MPTraits>::PositionDistance(env, diff, 2, .5, true);
	int nTicksPos=floor(d_pos/_posRes);
        double d_rot=RVDistance<MPTraits>::RotationalDistance(diff, 2);
        int nTicksRot=floor(d_rot/_oriRes);
	diff.ResetRigidBodyCoordinates();
	nTicks=max(nTicksPos,nTicksRot);
	incriment=diff.GetData();
	for(int i=0; i<incriment.size();i++)
	  incriment[i]=incriment[i]/nTicks;
	tick_num=0;
	while(tick_num<nTicks*m_S){
	  tick_num++;
	  vector<double> cfgData=tick.GetData();
	  for(int i=0; i<6;i++){
	    cfgData[i]+=incriment[i];
	  }
	  if(cfgData.size()==0){
	    return false;
	  }
	  tick.SetData(cfgData);
	  if(!env->InBounds(tick) || !vcm->IsValid(tick, cdInfo, callee)) {
            return false;
          }
	}
      }

      //use rv perturbation to step through each joint position
      //get joint positions of _c1
      vector<Vector3d> joints2;
      CfgType c2Copy = _c2;
      c2Copy.ResetRigidBodyCoordinates();
      c2Copy.ConfigureRobot();
      env->GetRobot(c2Copy.GetRobotIndex())->PolygonalApproximation(joints2);
      queue<shared_ptr<ReachableVolumeJointTreeNode> > joints;
      joints.push(m_rvr.m_RVLinkages.front()->m_root);
      while(!joints.empty()){
	//step through front
	bool stop=false;
	int jid=joints.front()->m_jointID;
	if(jid==0 || jid==1){
	  for(int i=0; i<joints.front()->m_children.size(); i++){
	    joints.push(joints.front()->m_children[i]);
	  }
	  joints.pop();
	  continue;
	}

	while(!stop){
	  CfgType tickCopy=tick;
	  tickCopy.ResetRigidBodyCoordinates();
	  tickCopy.ConfigureRobot();
	  vector<Vector3d> jointsTick;
	  env->GetRobot(tickCopy.GetRobotIndex())->PolygonalApproximation(jointsTick);
	  double d_joints=ReachableVolume::distance(jointsTick[jid],joints2[jid]);
	  double rvRes=m_rvres;
	  if(d_joints<=m_rvres*2){
	    stop=true;
	    rvRes=d_joints;
	  }
	  m_rvr.perturbJoint(jointsTick, jid, joints2[jid],rvRes,m_repositionPolicy);

	  vector<Vector3d> newNode;
	  newNode.resize(jointsTick.size());
	  for(int i=0; i<jointsTick.size();i++){
	    newNode[i]=jointsTick[i];
	  }

	  vector<double> *jointAngleCfg = this->m_rvr.convertToJointAngleSample(newNode);
          if(jointAngleCfg->size()==0){
	    delete(jointAngleCfg);
            return false;
	  }

	  tick.SetJointData(*jointAngleCfg);
	  delete(jointAngleCfg);
	  tick.ConfigureRobot();
	  if(!env->InBounds(tick) || !vcm->IsValid(tick, cdInfo, callee)) {
	    return false;
	  }
	}
	int last = -1;
	for(int i=0; i<joints.front()->m_children.size(); i++){
	  if(joints.front()->m_jointID==-1 || joints.front()->m_jointID==last)
	    continue;
	  joints.push(joints.front()->m_children[i]);
	  last=joints.front()->m_jointID;
	}
	joints.pop();
      }

      if(!m_rvr.m_fixed){
	while(tick_num<nTicks){
          tick_num++;
	  vector<double> cfgData=tick.GetData();
	  for(int i=0; i<6;i++){
            cfgData[i]+=incriment[i];
          }
          if(cfgData.size()==0){
            return false;
	  }
          tick.SetData(cfgData);
	  if(!env->InBounds(tick) || !vcm->IsValid(tick, cdInfo, callee)) {
            return false;
          }
        }
      }
      return true;
    }



    template<typename EnableCfg>
      EnableCfg DifferenceCfg(const EnableCfg& _c1, const EnableCfg& _c2,
			      typename boost::disable_if<IsClosedChain<EnableCfg> >::type* _dummy = 0) {
      return _c1 - _c2;
    }

    string m_vcLabel;
};

template<class MPTraits>
RVLocalPlanner<MPTraits>::RVLocalPlanner(string _vcLabel, bool _evalation) :
  LocalPlannerMethod<MPTraits>(), m_vcLabel(_vcLabel) {
    this->SetName("RVLocalPlanner");
    m_rvr.loadTree("Chain.tree",m_treeStructure);
  }



template<class MPTraits>
RVLocalPlanner<MPTraits>::RVLocalPlanner(MPProblemType* _problem, XMLNode& _node) :
  LocalPlannerMethod<MPTraits>(_problem, _node) {
    this->SetName("RVLocalPlanner");
    m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
    m_treeStructure = _node.Read("treeStructure", false, "EndEffectorFirst", "Structure of reachable volume tree");
    this->m_repositionPolicy = _node.Read("repositionPolicy", false, "Random", "Policy for repositioning children that are outside of reachable volume");
    m_rvres=_node.Read("rvres", false, .05, (double)0, (double)1000, "Resolution for reachable volume stepping");
    m_S = _node.Read("S", false, .5, 0.0, 1.0, "S, the scaling factor used by RV distance metric d=S*TranslationalDistance + (1-S)*RVDistance");

    m_rvr.loadTree("Chain.tree",m_treeStructure);
  }

template<class MPTraits>
RVLocalPlanner<MPTraits>::~RVLocalPlanner() { }

template<class MPTraits>
void
RVLocalPlanner<MPTraits>::PrintOptions(ostream& _os) {
  _os << "    " << this->GetName() << "::  ";
  _os << "vcLabel = " << " " << m_vcLabel << " ";
  _os << endl;
}

#endif
