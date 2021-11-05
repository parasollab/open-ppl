#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#undef PI

#include <trajectory_msgs/JointTrajectory.h>

using mathtool::Vector3d;
using mathtool::Transformation;
using mathtool::EulerAngle;


#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<double> ROSStepFunction::s_jointStates = {};

/*-------------------------- Construction -----------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node)
              : FollowPath(_agent,_node), m_ac("move_base", true){

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

    // ros::start();

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

  // std::cout << "Checking if waypoint: " << _waypoint.PrettyPrint() <<  "was reached" << std::endl;
  bool reached = false;
  if(m_robotLabel == "ur5") {
    reached = ReachedWaypointArm(_waypoint);
  }

  if(m_robotLabel == "boxer") {
    // reached = ReachedWaypointBase(_waypoint);
    reached = m_reachedWaypoint;
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
    auto jv = d/(KDL::PI);
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
  bool reached = false;
  // std::cout << "Current State: " << m_ac.getState().getText() << std::endl;
  if(m_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved 1 meter forward");
    reached = true;
  }else {
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
  return reached;
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
    MoveBase(_waypoint, _dt);
  }
}

void
ROSStepFunction::
MoveBase(const Cfg& _goal, double _dt) {

  // typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  //tell the action client that we want to spin a thread by default
  // MoveBaseClient ac("move_base", true);



  //Roll pitch and yaw in Radians
  // std::cout << "Goal: " << _goal << std::endl;
  // std::cout << "Goal[2]: " << _goal[2] << std::endl;
  // Compute the relative rotation from here to there in the local frame.
  Quaternion q;
  convertFromEulerVector(q, {_goal[2]*KDL::PI, 0.0, 0.0});
  auto real = q.real();
  auto vectorImaginary = q.imaginary();

  // std::cout << "Quat: " << real << " " << vectorImaginary << std::endl;

  // tf2::Quaternion q2;
  // q2.setRPY(0, 0, _goal[2]*KDL::PI);
  // std::cout << "Q2: " << q2.w() << " " << q2.x() << " " << q2.y() << " " << q2.z() << std::endl;
  
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = _goal[0];
  goal.target_pose.pose.position.y = _goal[1];
  goal.target_pose.pose.orientation.x = vectorImaginary[2];
  goal.target_pose.pose.orientation.y = vectorImaginary[1];
  goal.target_pose.pose.orientation.z = vectorImaginary[0];
  goal.target_pose.pose.orientation.w = real;

  std::cout << goal << std::endl;

  // goal.target_pose.pose.position.x = -1.0;
  // goal.target_pose.pose.position.y = 0.5;
  // goal.target_pose.pose.orientation.x = 0;
  // goal.target_pose.pose.orientation.y = 0;
  // goal.target_pose.pose.orientation.z = 0;
  // goal.target_pose.pose.orientation.w = 1;

  ROS_INFO("Sending goal to boxer");

  m_ac.sendGoal(goal,
                MoveBaseClient::SimpleDoneCallback(),
                MoveBaseClient::SimpleActiveCallback(),
                boost::bind(&ROSStepFunction::SimpleFeedbackCallback, this, _1, _goal));

  m_ac.waitForResult(ros::Duration(1.0));

  // std::cout << "Wait Result: " << res << std::endl;
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
      msg.points[0].positions.push_back(d*KDL::PI);
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
  ROS_INFO_STREAM("Done Callback");
  // ROS_INFO("Status: %s", _state.getText().c_str());
  // ROS_INFO("State: %d", _state.state_);

  // // std::cout << "Status: " << _state.getText().c_str() << std::endl;

  // if(_state.state_ == 6) {
  //   m_reachedWaypoint = true;
  // }
}

void
ROSStepFunction::
SimpleFeedbackCallback(const move_base_msgs::MoveBaseFeedback::ConstPtr& _feedback, const Cfg& _waypoint) {
  // ROS_INFO("FEEDBACK");
  // std::cout << "Waypoint: " << _waypoint << std::endl;
  auto pose = _feedback->base_position;
  // std::cout << "Pose: " << pose << std::endl;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // geometry_msgs::TransformStamped poseTrans;

  try {
    // auto poseTrans = tfBuffer.transform(pose, "map");
    geometry_msgs::TransformStamped odom_to_map = tfBuffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(0.5));
    tf2::doTransform(pose, pose, odom_to_map);
    // std::cout << "Tranformed Pose: " << pose << "\n" << std::endl;

  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
  std::cout << "Pose: " << pose << std::endl;

  Quaternion q(pose.pose.orientation.w, {pose.pose.orientation.z, pose.pose.orientation.y, pose.pose.orientation.x});
  //q.normalize();

  EulerAngle e;
  mathtool::convertFromQuaternion(e, q);

  Cfg cur(this->m_agent->GetRobot());
  cur[0] = pose.pose.position.x;
  cur[1] = pose.pose.position.y;
  cur[2] = e.gamma() / KDL::PI;


  // std::cout << "Alpha: " << e.alpha() / KDL::PI << std::endl;
  // std::cout << "Beta: " << e.beta() / KDL::PI << std::endl;
  // std::cout << "Gamma: " << e.gamma() / KDL::PI << std::endl;
  // cur[2] = pose.pose.position.z;
  // cur[3] = e.alpha();
  // cur[4] = e.beta();
  // cur[5] = e.gamma();

  // std::cout << "Cur: " << cur << std::endl;
  m_reachedWaypoint = _waypoint.WithinResolution(cur, 0.05, 1.0);
  // std::cout << "Cur: " << cur << std::endl;
}