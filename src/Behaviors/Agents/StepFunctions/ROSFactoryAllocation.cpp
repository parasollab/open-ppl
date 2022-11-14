#include "ROSFactoryAllocation.h"

#include <boost/bind.hpp>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>

/*------------------------------ Construction --------------------------------*/
ROSFactoryAllocation::
ROSFactoryAllocation(Agent* _agent, XMLNode& _node) : StepFunction(_agent,_node) {

  ros::start();

  ros::NodeHandle nh;

  m_timeThreshold = _node.Read("timeThreshold",false,20.,0.,MAX_DBL,
      "Time remaining threshold to add to allocation.");

  for(auto& child : _node) {
    if(child.Name() == "TaskPoint") {
      ParseTaskPoint(child,nh);
    }
  }

  auto prob = _agent->GetRobot()->GetMPProblem();
  for(auto& robot : prob->GetRobots()) {
    auto name = robot->GetLabel();
    auto topic = "/" + name + "/task_queue";
    m_taskQueuePubs[name] = nh.advertise<geometry_msgs::PoseStamped>(topic, 1, true);
  }

}

ROSFactoryAllocation::
~ROSFactoryAllocation() { }

/*-------------------------------- Interface ---------------------------------*/

void
ROSFactoryAllocation::
StepAgent(double _dt) {

  for(auto kv : m_taskPoints) {
    auto label = kv.first;
    UpdateTimeRemaining(label);
  }

  auto decomp = BuildDecomposition();

  if(decomp.get()) {
    m_decompositions.push_back(std::move(decomp));
    PlanAllocations(m_decompositions.back().get());
  }

  // Temporary
  for(auto kv : m_timeRemaining) {

    auto label = kv.first;

    if(m_allocated.count(label))
      continue;

    if(kv.second <= m_timeThreshold) {
      AssignTask("robot1",label,m_taskPoints[label].location);
    }
  }
}

/*---------------------------- Helper Functions ------------------------------*/

void
ROSFactoryAllocation::
ParseTaskPoint(XMLNode& _node, ros::NodeHandle& _nh) {

  TaskPoint tp;
  tp.label = _node.Read("label",true,"","Label for task point.");
  tp.topic = _node.Read("topic",true,"","Topic to track time remaining.");

  std::string point = _node.Read("location",true,"","Location of the task point.");
  std::istringstream buffer(point);

  double d;
  for(size_t i = 0; i < 3; i++) {
    buffer >> d;
    tp.location.push_back(d);
  }

  m_taskPoints[tp.label] = tp;

  //m_timeRemainingSubs[tp.label] = _nh.subscribe<std_msgs::Float32>(tp.topic, 1, 
  //    boost::bind(&ROSFactoryAllocation::TimeRemainingCallback,this,_1,tp.label));
  //m_timeRemainingSubs[tp.label] = _nh.subscribe(tp.topic, 1000, 
  //    TimeRemainingCallback);
}

void
ROSFactoryAllocation::
UpdateTimeRemaining(std::string _label) {

  auto topic = _label + "/" + m_taskPoints[_label].topic;
  auto sharedMsg = ros::topic::waitForMessage<std_msgs::Float32>(topic,ros::Duration(1));

  if(sharedMsg == NULL) {
    return;
  }

  auto msg = *sharedMsg;

  if(msg.data > m_timeRemaining[_label]) {
    m_allocated.erase(_label);
  }

  m_timeRemaining[_label] = msg.data;
}

void
ROSFactoryAllocation::
AssignTask(std::string _robot, std::string _label, std::vector<double> _location) {

  //auto topic = "/" + _robot + "/task_queue";

  std::cout << "ASSIGNING TASK " << _robot + " " + _label + " " << _location << std::endl;

  geometry_msgs::PoseStamped msg;
  msg.pose.position.x = _location[0];
  msg.pose.position.y = _location[1];
  msg.pose.position.z = _location[2];

  msg.header.frame_id = _label;

  std::cout << msg << std::endl;

  m_taskQueuePubs[_robot].publish(msg);

  m_allocated.insert(_label);
}
    
std::unique_ptr<Decomposition>
ROSFactoryAllocation::
BuildDecomposition() {

  
  //for(auto kv : m_timeRemaining) {
  //}
  

  return std::unique_ptr<Decomposition>(nullptr);
}

void
ROSFactoryAllocation::
PlanAllocations(Decomposition* _decomp) {

}

/*------------------------------ ROS Interface -------------------------------*/

/*----------------------------------------------------------------------------*/
