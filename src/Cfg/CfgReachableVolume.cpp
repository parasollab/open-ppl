#include "CfgReachableVolume.h"

#include "Environment/ActiveMultiBody.h"
#include "Environment/Environment.h"
#include "ReachableVolumeUtil/ReachableVolumeRobot.h"

shared_ptr<vector<shared_ptr<ReachableVolumeRobot> > > g_reachableVolumeRobots;

CfgReachableVolume::CfgReachableVolume()
  : Cfg() {
  if(g_reachableVolumeRobots){
    m_reachableVolumeRobots=g_reachableVolumeRobots;
  }else{
    m_reachableVolumeRobots = shared_ptr<vector<shared_ptr<ReachableVolumeRobot> > >(new vector<shared_ptr<ReachableVolumeRobot> >);
    for(int i=0; i<1; i++){
      m_reachableVolumeRobots->push_back(shared_ptr<ReachableVolumeRobot>(new ReachableVolumeRobot()));
      m_reachableVolumeRobots->back()->loadTree();
    }
    g_reachableVolumeRobots=m_reachableVolumeRobots;
  }
}

CfgReachableVolume::CfgReachableVolume(const Cfg&c)
  : Cfg(c){
}


CfgReachableVolume::CfgReachableVolume(const CfgReachableVolume&c)
  : Cfg(c){
  m_reachableVolumeRobots=c.m_reachableVolumeRobots;
  m_numOfJoints=c.m_numOfJoints;
}

void CfgReachableVolume::loadTreeFiles(){
  //set to load this from file
  //need to go back and change this o handle multiple robots + take file name as input
  for(int i=0; i<1; i++){
    (*m_reachableVolumeRobots)[i]->loadTree("Chain.tree");
  }
}

const string CfgReachableVolume::GetName() const {
  return "CfgReachableVolume";
}


void CfgReachableVolume::GetRandomCfg(Environment* _env){
  GetRandomCfg(_env,_env->GetBoundary());
}

void CfgReachableVolume::GetRandomCfg(Environment* _env, shared_ptr<Boundary> _bb){
  m_v.clear();
  for(size_t i = 0; i < m_robots.size(); ++i) {
    vector<double> cfg_data_robot;
    //whoever decided to make m_robots a vector of vectors is going to need to change this to correctly fit that format
    if(m_robots[i]->GetBaseType() == FreeBody::BodyType::Fixed ||
        (*m_reachableVolumeRobots)[i]->m_fixed){
      for(int j=0; j<6;j++)
        m_v.push_back(0);
    }
    else {
      if(m_robots[i]->GetBaseType() == FreeBody::BodyType::Planar ||
          m_robots[i]->GetBaseType() == FreeBody::BodyType::Volumetric) {
        Point3d p=_bb->GetRandomPoint();
        size_t posDOF = m_robots[i]->GetBaseType() == FreeBody::BodyType::Volumetric ? 3 : 2;
        for(size_t j = 0; j < posDOF; j++){
	  (*m_reachableVolumeRobots)[i]->m_baseJointPos[j]=p[j];
          m_v.push_back(p[j]);
        }
        if(m_robots[i]->GetBaseMovementType() == FreeBody::MovementType::Rotational) {
          size_t oriDOF = m_robots[i]->GetBaseType() == FreeBody::BodyType::Volumetric ? 3 : 1;
          for(size_t i = 0; i < oriDOF; i++) {
	    m_v.push_back(DRand());
          }
        }
      }
    }
    shared_ptr<vector<double> > internalCfg = (*m_reachableVolumeRobots)[i]->getInternalCFGCoordinates();
    if(m_robots[i]->GetBaseType() == FreeBody::BodyType::Planar)
      (*internalCfg)[1]=DRand();
    m_v.insert(m_v.end(), internalCfg->begin(), internalCfg->end());
  }

  if((*m_reachableVolumeRobots)[0]->m_debug) {
    cout<<"cfg ="<<endl;
    for(size_t i = 0; i < m_v.size(); i++)
      cout<<m_v[i]<<endl;
    Cfg::SetData(m_v);
    ConfigureRobot();
    /*
    _env->GetMultiBody(_env->GetRobotIndex())->PolygonalApproximation(joints);
    cout<<"************"<<endl<<"Printing Joints"<<endl;
    for(int i =0; i<joints.size(); i++){
      cout<<joints[i]<<endl;
    }
    cout<<"************"<<endl<<"Printing Link Lengths"<<endl;
    for(int i =0; i<joints.size()-1; i++){
      cout<<ReachableVolume::distance(joints[i], joints[i+1])<<endl;
    }
    cout<<"************"<<endl;
    */
  }
  /*
  for(int i=0; i<m_robots.size(); i++)
    (*m_reachableVolumeRobots)[i].reset();
  m_reachableVolumeRobots->clear();
  */
}

/*
//function not used by current version of the code
vector<Robot> CfgReachableVolume::GetRobots(vector<Robot> &_robots, const Environment* _env) {
  m_numOfJoints = 0;
  vector<Robot> robots;
  int j=0;
  vector<Robot>::iterator rit=_robots.begin();
  for(vector<shared_ptr<ReachableVolumeRobot> >::iterator iter = m_reachableVolumeRobots->begin(); iter!= m_reachableVolumeRobots->end(); iter++){
    Robot::JointMap joints;
    int linkage_joints;
    linkage_joints=(*iter)->m_nLinks-1;
    m_numOfJoints += linkage_joints;
    unsigned int nextEarLink=0;
    shared_ptr<MultiBody> mb = _env->GetMultiBody(rit->m_bodyIndex);
    for(int i = 0; i<linkage_joints; i++){
      if(nextEarLink>=(*iter)->m_earParentLinks.size()){
	if((*m_reachableVolumeRobots)[0]->m_debug) cout<<"Adding joint to robot "<<i<<" "<<i+1<<endl;
	shared_ptr<Connection> c = shared_ptr<Connection>(new Connection(mb->GetFreeBody(i),mb->GetFreeBody(i+1)));
	c->SetConnectionType(Connection::SPHERICAL);
	joints.push_back(c);
      }else{
	if((*iter)->m_earLinks[nextEarLink]==i+1){
	  if((*m_reachableVolumeRobots)[0]->m_debug) cout<<"Adding joint to robot "<<(*iter)->m_earParentLinks[nextEarLink]<<" "<<i+1<<endl;
	  shared_ptr<Connection> c = shared_ptr<Connection>(new Connection(mb->GetFreeBody((*iter)->m_earParentLinks[nextEarLink]), mb->GetFreeBody(i+1)));
	  c->SetConnectionType(Connection::SPHERICAL);
	  joints.push_back(c);
	  nextEarLink++;
	}else{
	  if((*m_reachableVolumeRobots)[0]->m_debug) cout<<"Adding joint to robot "<<i<<" "<<i+1<<endl;
	  shared_ptr<Connection> c = shared_ptr<Connection>(new Connection(mb->GetFreeBody(i),mb->GetFreeBody(i+1)));
	  c->SetConnectionType(Connection::SPHERICAL);
	  joints.push_back(c);
	}
      }
    }
    size_t _robotIndex = 0;  //assuming robot index = 0;
    size_t _robotBaseIndex = 0;  //assuming robot base index = 0;
    robots.push_back(Robot(_robots[j].m_base, _robots[j].m_baseMovement, joints, 0,_env->GetActiveBody(_robotIndex)->GetFreeBody(_robotBaseIndex)));   //assumes base is 0
    j++;
    rit++;
  }
  return robots;
}

//function not used by current version of the code
vector<Robot> CfgReachableVolume::GetRobots(int _numJoints) {
  m_numOfJoints = _numJoints;
  vector<Robot> robots;
  return robots;
}
*/
