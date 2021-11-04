#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#include <trajectory_msgs/JointTrajectory.h>

using mathtool::Vector3d;
using mathtool::Transformation;
using mathtool::EulerAngle;

std::vector<double> ROSStepFunction::s_jointStates = {};

/*-------------------------- Construction -----------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node)
              : FollowPath(_agent,_node), m_ac("move_base", true) {

  m_time = _node.Read("time",false,m_time,0.,10.,
                      "Time for executing controls.");

  m_sim = _node.Read("sim",false,m_sim,"Flag indiciating if this is a simulated robot or not.");

  m_robotLabel = _node.Read("robotLabel", true, "", "The robot to be controlled.");

  // int argc = 0;
  // char* argv[255];

  // ros::init(argc,argv,"ppl_" + this->m_agent->GetRobot()->GetLabel());

  if(m_robotLabel == "ur5") {

    ros::start();

    ROS_INFO_STREAM("Hello Worlid!! --- ur5 Mode");

    ros::NodeHandle nh;

    if(m_sim) {
      m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                  //"/arm_controller/command",
                  "/pos_joint_traj_controller/command",
                  //"/"+m_agent->GetRobot()->GetLabel()+"/arm_controller/command",
                  10);
    }
    else {
      m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                  //"/arm_controller/command",
                  "/scaled_pos_joint_traj_controller/command",
                  //"/"+m_agent->GetRobot()->GetLabel()+"/arm_controller/command",
                  10);
    }

    JointStateCallback callback = [this](const sensor_msgs::JointState _msg) {
      this->Callback(_msg);
    };

    //m_stateSub = nh.subscribe(m_agent->GetRobot()->GetLabel()+"/joint_states",
    //                          1000,Callback);
  }

  if(m_robotLabel == "boxer") {

    ros::start();

    ROS_INFO_STREAM("Hello Worlid!! --- Boxer Mode");

    //wait for the action server to come up
    while(!m_ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    } 
    
  }
}

ROSStepFunction::
~ROSStepFunction() { }

/*---------------------------- Interface ---------------------------*/

/*------------------------- Helper Functions -----------------------*/

bool
ROSStepFunction::
ReachedWaypoint(const Cfg& _waypoint) {

  std::cout << "Checking if waypoint: " << _waypoint.PrettyPrint() <<  "was reached" << std::endl;
  bool reached = false;
  if(m_robotLabel == "ur5") {
    reached = ReachedWaypointArm(_waypoint);
  }

  if(m_robotLabel == "boxer") {
    reached = ReachedWaypointBase(_waypoint);
    if(reached)
      ROS_INFO_STREAM("Reached: true");
    else
      ROS_INFO_STREAM("Reached: false");
  }
  return reached;
}

bool
ROSStepFunction::
ReachedWaypointArm(const Cfg& _waypoint) {
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
    //auto jv = d/(2*PI);
    auto jv = d/(PI);
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


bool
ROSStepFunction::
ReachedWaypointBase(const Cfg& _waypoint) {

  // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  //tell the action client that we want to spin a thread by default
  // MoveBaseClient ac("move_base", true);
  // if(m_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
  //   ROS_INFO("Hooray, the base moved 1 meter forward");
  //   m_reachedWaypoint = true;
  // }else {
  //   ROS_INFO("The base failed to move forward 1 meter for some reason");
  // }
  return m_reachedWaypoint;
}

void
ROSStepFunction::
MoveToWaypoint(const Cfg& _waypoint, double _dt) {
  std::cout << "Moving to waypoint: " << _waypoint.PrettyPrint() << std::endl;

  if(m_robotLabel == "ur5") {
    //ROS HACK BC WRIST 3 IS NOISY AND DUMB
    auto hack = _waypoint;
    hack[5] = 0;

    MoveArm(_waypoint.GetData(), _dt);
  }

  if(m_robotLabel == "boxer") {
    MoveBase(_waypoint.GetData(), _dt);
  }
}

void
ROSStepFunction::
MoveBase(std::vector<double> _goal, double _dt) { 

  // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  //tell the action client that we want to spin a thread by default
  // MoveBaseClient ac("move_base", true);

  

  //Roll pitch and yaw in Radians
  double roll = _goal[3],
         pitch = _goal[4],
         yaw = _goal[5];
  // Compute the relative rotation from here to there in the local frame.
  Quaternion q;
  convertFromEulerVector(q,{roll,pitch,yaw});
  auto real = q.real();
  auto vectorImaginary = q.imaginary();

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = _goal[0];
  goal.target_pose.pose.position.y = _goal[1];
  goal.target_pose.pose.orientation.x = vectorImaginary[0];
  goal.target_pose.pose.orientation.y = vectorImaginary[1];
  goal.target_pose.pose.orientation.z = vectorImaginary[2];
  goal.target_pose.pose.orientation.w = real;

  ROS_INFO("Sending goal to boxer");
  std::cout << _goal << std::endl;

  m_ac.sendGoal(goal,
                boost::bind(&ROSStepFunction::SimpleDoneCallback, this, _1, _2),
                MoveBaseClient::SimpleActiveCallback(),
                boost::bind(&ROSStepFunction::SimpleFeedbackCallback, this, _1));
  
  auto res = m_ac.waitForResult(ros::Duration(5.0));
  std::cout << "Wait Result: " << res << std::endl;
  // m_ac.cancelGoal();

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
      //msg.points[0].positions.push_back(d*2*PI);
      msg.points[0].positions.push_back(d*PI);
    }

    msg.points[0].time_from_start = ros::Duration(m_time);

    /*
    ROS_INFO_STREAM("Sending command:\n" << msg);
    m_armPub.publish(msg);
    rate.sleep();
    m_armPub.publish(msg);
    rate.sleep();
    m_armPub.publish(msg);
    rate.sleep();
    */
  }
}

void
ROSStepFunction::
Callback(const sensor_msgs::JointState _msg) {
  ROS_INFO_STREAM("Received: " << _msg.position);
  s_jointStates = _msg.position;
}

void
ROSStepFunction::
SimpleDoneCallback(const actionlib::SimpleClientGoalState& _state, const move_base_msgs::MoveBaseResult::ConstPtr& _result) {
  ROS_INFO("Done Callback");
  // ROS_INFO("Status: %s", _state.getText().c_str());
  // ROS_INFO("State: %d", _state.state_);

  // // std::cout << "Status: " << _state.getText().c_str() << std::endl;  

  // if(_state.state_ == 6) {
  //   m_reachedWaypoint = true;
  // } 
}

void
ROSStepFunction::
SimpleFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& _feedback) {
  ROS_INFO("FEEDBACK");
  // ROS_INFO("Base Position x : %d",  _feedback->base_position.pose.position.x);
  std::cout << "Base Position x: " << _feedback->base_position.pose.position.x << std::endl;
  // auto pos = _feedback->base_position;
}