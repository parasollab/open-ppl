#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#include <trajectory_msgs/JointTrajectory.h>

std::vector<double> ROSStepFunction::s_jointStates = {};

/*-------------------------- Construction -----------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node) 
              : FollowPath(_agent,_node) {

  m_time = _node.Read("time",false,m_time,0.,10.,
                      "Time for executing controls.");

  int argc = 0;
  char* argv[255];

  ros::init(argc,argv,"ppl_" + this->m_agent->GetRobot()->GetLabel());

  ros::start();
  
  ROS_INFO_STREAM("Hello World!");

  ros::NodeHandle nh;
  m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                "/scaled_pos_joint_traj_controller/command",
                //"/"+m_agent->GetRobot()->GetLabel()+"/arm_controller/command",
                10);

  /*JointStateCallback callback = [this](const sensor_msgs::JointState _msg) {
    this->Callback(_msg);
  };*/

  //m_stateSub = nh.subscribe(m_agent->GetRobot()->GetLabel()+"/joint_states",
  //                          1000,Callback);
}

ROSStepFunction::
~ROSStepFunction() { }

/*---------------------------- Interface ---------------------------*/

/*------------------------- Helper Functions -----------------------*/

bool 
ROSStepFunction::
ReachedWaypoint(const Cfg& _waypoint) {
  //ros::spinOnce();

  // Grab joint angles
  //auto js = s_jointStates;

  sensor_msgs::JointState msg;
  //auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/"+m_agent->GetRobot()->GetLabel()+"/joint_states");
  auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

  if(sharedMsg!=NULL)
    msg = *sharedMsg;
  ROS_INFO_STREAM(m_agent->GetRobot()->GetLabel() << " Received: " << msg.position);

  auto js = msg.position;

  // Check to make sure they have been set
  if(js.size() == 0)
    return false;

  // Convert joint angles to pmpl representation
  std::vector<double> jointStates;
  for(auto d : js) {
    auto jv = d/(2*PI);
    //Hack to deal with nonsense ros joint status values that forget about joint limits
    if(jv > 1)
      jv = -2 + jv;
    jointStates.push_back(jv);
  }
  
  //TODO::Check if m_jointState has reached the current waypoint
  auto r = m_agent->GetRobot();
  Cfg state(r);
  state.SetData(jointStates);
  
  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto lib = p->GetMPLibrary();
  auto dm = lib->GetDistanceMetric(m_waypointDm);

  //Hack to deal with noisy wrist joint
  auto waypoint = _waypoint;
  waypoint[5] = state[5];

  double distance = dm->Distance(state, waypoint);

  std::cout << "Robot: " << m_agent->GetRobot()->GetLabel()
            << "\n\tCurrent: " << state.PrettyPrint()
            << "\n\tWaypoint: " << waypoint.PrettyPrint()
            << "\n\tDistance: " << distance << std::endl;

  return distance < m_waypointThreshold;
}

void 
ROSStepFunction::
MoveToWaypoint(const Cfg& _waypoint, double _dt) {
  std::cout << "Moving to waypoint: " << _waypoint.PrettyPrint() << std::endl;
  MoveArm(_waypoint.GetData(), _dt);
}
    
void 
ROSStepFunction::
MoveArm(std::vector<double> _goal, double _dt) {
  if(ros::ok()) {

    ros::Rate rate(3);

    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    /*msg.joint_names.push_back("sphere_0_to_link_1");
    msg.joint_names.push_back("sphere_1_to_link_2");
    msg.joint_names.push_back("sphere_2_to_link_3");
    */

    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");

    msg.points.resize(1);

    //msg.points[0].positions = _goal;
    for(auto d : _goal) {
      msg.points[0].positions.push_back(d*2*PI);
    }

    msg.points[0].time_from_start = ros::Duration(m_time);

    ROS_INFO_STREAM("Sending command:\n" << msg);
    m_armPub.publish(msg);
    rate.sleep();
    /*m_armPub.publish(msg);
    rate.sleep();
    m_armPub.publish(msg);
    rate.sleep();*/
  }
}
   
void 
ROSStepFunction::
Callback(const sensor_msgs::JointState _msg) {
  ROS_INFO_STREAM("Received: " << _msg.position);
  s_jointStates = _msg.position;
}
