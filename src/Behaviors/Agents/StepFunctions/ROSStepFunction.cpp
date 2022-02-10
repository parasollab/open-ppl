#include "ROSStepFunction.h"

#include "Behaviors/Agents/PathFollowingAgent.h"

#include "MPProblem/Robot/Robot.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <geometry_msgs/Twist.h>

#undef PI

using mathtool::Vector3d;
using mathtool::Transformation;
using mathtool::EulerAngle;

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <sstream>

std::vector<double> ROSStepFunction::s_jointStates = {};

/*-------------------------- Construction -----------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node)
              : FollowPath(_agent,_node) {

  m_time = _node.Read("time",false,m_time,0.,10.,
                      "Time for executing controls.");

  m_sim = _node.Read("sim",false,m_sim,"Flag indiciating if this is a simulated robot or not.");

  m_robotLabel = _node.Read("robotLabel", true, "", "The robot to be controlled.");

  std::string jointNames = _node.Read("jointNames",true,"","The set of joints to control.");

  // Convert string list to vector of strings
  std::istringstream ss(jointNames);
  std::string joint;
  while(ss >> joint) {
    m_jointNames.push_back(joint);
  }

  auto compareFunc = [](std::string _a, std::string _b) {
    return _a < _b;
  };
  std::sort(m_jointNames.begin(),m_jointNames.end(),compareFunc);

  ros::start();

  ros::NodeHandle nh;
  if(m_robotLabel == "ur5") {

    ROS_INFO_STREAM("Hello Worlid!! --- ur5 Mode");


    if(m_sim) {
      m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                  //"/arm_controller/command",
                  "/" + this->m_agent->GetRobot()->GetLabel()+"/pos_joint_traj_controller/command",
                  //"/"+m_agent->GetRobot()->GetLabel()+"/arm_controller/command",
                  10);
    }
    else {
      m_armPub = nh.advertise<trajectory_msgs::JointTrajectory>(
                  "/" + this->m_agent->GetRobot()->GetLabel() + "/ppl_joint_traj_controller/command",
                  10);
    }

    JointStateCallback callback = [this](const sensor_msgs::JointState _msg) {
      this->Callback(_msg);
    };

    //m_stateSub = nh.subscribe(m_agent->GetRobot()->GetLabel()+"/joint_states",
    //                          1000,Callback);
  }

  if(m_robotLabel == "boxer") {

    ROS_INFO_STREAM("Hello Worlid!! --- Boxer Mode");

    std::string topic = this->m_agent->GetRobot()->GetLabel() + "/twist_marker_server/cmd_vel";

    m_boxerPub = nh.advertise<geometry_msgs::Twist>(
                 topic,
                 10);

    BoxerOdomCallback boxer_callback = [this](const nav_msgs::Odometry _msg) {
      this->OdomCallback(_msg);
    };

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
    reached = ReachedWaypointBase(_waypoint);
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

  if(m_sim) {
    auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/"+m_agent->GetRobot()->GetLabel()+"/joint_states");
    //auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ppl_joint_states");

    if(sharedMsg!=NULL)
      msg = *sharedMsg;
  }
  else {
    auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/"+m_agent->GetRobot()->GetLabel()+"/ppl_joint_states");
    //auto sharedMsg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ppl_joint_states");

    if(sharedMsg!=NULL)
      msg = *sharedMsg;
  }

  ROS_INFO_STREAM(m_agent->GetRobot()->GetLabel() << " Received: " << msg.position);

  auto js = msg.position;

  // Check to make sure they have been set
  if(js.size() == 0)
    return false;

  // Convert joint angles to pmpl representation
  /*std::vector<double> jointStates;
  for(auto d : js) {
    //auto jv = d/(2*PI);
    auto jv = d/(KDL::PI);
    //Hack to deal with nonsense ros joint status values that forget about joint limits
    if(jv > 1)
      jv = -2 + jv;
    jointStates.push_back(jv);
  }*/

  //TODO::Sort alphabetically by corresponding joint names

  std::vector<double> jointStates;
  auto iter = js.begin();
  for(auto& joint : this->m_agent->GetRobot()->GetMultiBody()->GetJoints()) {
    switch(joint->GetConnectionType()) {
      case Connection::JointType::NonActuated:
      {
        break;
      }
      case Connection::JointType::Mimic:
      {
        if(m_sim) {
          iter++;
        }
        break;
      }
      case Connection::JointType::Spherical:
      {
        throw RunTimeException(WHERE) << "Spherical joints not supported here.";
      }
      default: 
      {
        auto jv = (*iter)/(KDL::PI);
        //Hack to deal with nonsense ros joint status values that forget about joint limits
        if(jv > 1)
          jv = -2 + jv;
        else if(jv < -1)
          jv = 2 + jv;
        jointStates.push_back(jv);
        iter++;
      }
    }
    if(iter == js.end())
      break;
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

  auto state = GetCurrentState();

  auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
  auto lib = p->GetMPLibrary();
  auto dm = lib->GetDistanceMetric(m_waypointDm);

  double distance = dm->Distance(state, _waypoint);

  std::cout << "Robot: " << m_agent->GetRobot()->GetLabel()
            << "\n\tCurrent: " << state.PrettyPrint()
            << "\n\tWaypoint: " << _waypoint.PrettyPrint()
            << "\n\tDistance: " << distance << std::endl;
  std::cout << "Distance: " << (distance < m_waypointThreshold) << std::endl;

  return distance < m_waypointThreshold;
}

void
ROSStepFunction::
MoveToWaypoint(const Cfg& _waypoint, double _dt) {
  std::cout << "Moving to waypoint: " << _waypoint.PrettyPrint() << std::endl;

  if(m_robotLabel == "ur5") {
    //ROS HACK BC WRIST 3 IS NOISY AND DUMB
    //auto hack = _waypoint;
    //hack[5] = 0;

    MoveArm(_waypoint.GetData(), _dt);
  }

  if(m_robotLabel == "boxer") {
    MoveBase(_waypoint, _dt);
  }
}

void
ROSStepFunction::
MoveBase(const Cfg& _goal, double _dt) {
  if(ros::ok()) {
    auto p = dynamic_cast<PathFollowingAgent*>(m_agent);
    auto robot = p->GetRobot();

    // Get current state of the system from ROS
    const Cfg current = GetCurrentState();

    // Get the best control to take system from the current state to the goal
    // state.
    auto control = robot->GetController()->operator()(current, _goal, 1.0).GetOutput();

    /// TODO: This needs to become generic to the message type.
    geometry_msgs::Twist msg;

    msg.linear.x = control[0];
    msg.linear.y = control[1];
    msg.angular.z = control[2] * KDL::PI;

    m_boxerPub.publish(msg);
  }
}

void
ROSStepFunction::
MoveArm(std::vector<double> _goal, double _dt) {

  if(_goal == m_previousGoal)
    return;

  m_previousGoal = _goal;

  if(ros::ok()) {

    ros::Rate rate(3);

    trajectory_msgs::JointTrajectory msg;

    msg.joint_names.clear();
    /*msg.joint_names.push_back("sphere_0_to_link_1");
    msg.joint_names.push_back("sphere_1_to_link_2");
    msg.joint_names.push_back("sphere_2_to_link_3");
    */

    /*msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");*/

    for(auto joint : m_jointNames) {
      msg.joint_names.push_back(joint);
    }

    msg.points.resize(1);

    //msg.points[0].positions = _goal;
    /*for(auto d : _goal) {
      //msg.points[0].positions.push_back(d*2*PI);
      msg.points[0].positions.push_back(d*KDL::PI);
    }*/

    auto iter = _goal.begin();
    std::unordered_map<Connection*,size_t> mimicJointMap;
    std::unordered_map<Connection*,double> parentJointValueMap;

    for(auto& joint : this->m_agent->GetRobot()->GetMultiBody()->GetJoints()) {
      switch(joint->GetConnectionType()) {
        case Connection::JointType::NonActuated:
          {
            break;
          }
        case Connection::JointType::Mimic:
          {
            // Add placeholder value for mimic joint
            mimicJointMap[joint.get()] = msg.points[0].positions.size();
            msg.points[0].positions.push_back(0);
            break;
          }
        case Connection::JointType::Spherical:
          {
            throw RunTimeException(WHERE) << "Spherical joints not supported here.";
          }
        default: 
          {
            double value = (*iter)*KDL::PI;
            msg.points[0].positions.push_back(value);
            parentJointValueMap[joint.get()] = value;
            iter++;
          }
      }
    }

    // Backfill mimic joint values
    for(auto kv : mimicJointMap) {
      auto parent = kv.first->GetMimicConnection();
      auto value = parentJointValueMap[parent];
      auto relation = kv.first->GetMimicRelationship();
      value = (value * relation.first) + relation.second;;
      msg.points[0].positions[kv.second] = value;
    }

    msg.points[0].time_from_start = ros::Duration(m_time);

    ROS_INFO_STREAM("Sending command:\n" << msg);
    m_armPub.publish(msg);
    rate.sleep();
    //m_armPub.publish(msg);
    //rate.sleep();
    //m_armPub.publish(msg);
    //rate.sleep();
  }
}

Cfg
ROSStepFunction::
GetCurrentState() {
  // Wait for an odometry message from ROS
  nav_msgs::Odometry msg;
  std::string topic = this->m_agent->GetRobot()->GetLabel() + "/odometry/filtered";
  auto sharedMsg = ros::topic::waitForMessage<nav_msgs::Odometry>(topic);

  if(sharedMsg != NULL)
    msg = *sharedMsg;

  auto pose = msg.pose.pose;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  try {
    geometry_msgs::TransformStamped odom_to_map = tfBuffer.lookupTransform("map", this->m_agent->GetRobot()->GetLabel() + "/odom", ros::Time(0), ros::Duration(0.5));
    tf2::doTransform(pose, pose, odom_to_map);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  Quaternion q(pose.orientation.w, {pose.orientation.z, pose.orientation.y, pose.orientation.x});
  EulerAngle e;
  mathtool::convertFromQuaternion(e, q);

  auto r = m_agent->GetRobot();
  Cfg state(r);
  state[0] = pose.position.x;
  state[1] = pose.position.y;
  state[2] = e.gamma() / KDL::PI;

  return state;
}

void
ROSStepFunction::
Callback(const sensor_msgs::JointState _msg) {
  ROS_INFO_STREAM("Received: " << _msg.position);
  s_jointStates = _msg.position;
}

void
ROSStepFunction::
OdomCallback(const nav_msgs::Odometry _msg) {
  ROS_INFO_STREAM("Received: " << _msg.pose.pose);
}
