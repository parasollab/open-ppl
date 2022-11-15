#include "ROSFactoryAllocation.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"
#include "MPProblem/MPTask.h"

#include <boost/bind.hpp>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>

/*------------------------------ Construction --------------------------------*/
ROSFactoryAllocation::
ROSFactoryAllocation(Agent* _agent, XMLNode& _node) : StepFunction(_agent,_node) {

  ros::start();

  ros::NodeHandle nh;

  for(auto& child : _node) {
    if(child.Name() == "TaskPoint") {
      ParseTaskPoint(child,nh);
    }
  }

  std::string depot = _node.Read("depot",true,"","Location of the depot.");
  std::istringstream buffer(depot);

  double d;
  for(size_t i = 0; i < 3; i++) {
    buffer >> d;
    m_depot.push_back(d);
  }

  auto prob = _agent->GetRobot()->GetMPProblem();
  for(auto& robot : prob->GetRobots()) {
    auto name = robot->GetLabel();
    auto topic = "/" + name + "/task_queue";
    m_taskQueuePubs[name] = nh.advertise<geometry_msgs::PoseStamped>(topic, 1, true);
  }

  auto c = dynamic_cast<Coordinator*>(this->m_agent);
  m_plan = std::unique_ptr<Plan>(new Plan());
  m_plan->SetCoordinator(c);
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

  // TODO::Sort into priority order
  std::vector<std::string> readyTasks;
  for(auto kv : m_countdown) {

    auto label = kv.first;

    if(m_allocated.count(label))
      continue;

    if(kv.second <= m_taskPoints[label].threshold) {
      readyTasks.push_back(label);
    }
  }

  auto decomp = BuildDecomposition(readyTasks);

  if(decomp.get()) {
    m_decompositions.push_back(std::move(decomp));
    PlanAllocations(m_decompositions.back().get());
  }

}

/*---------------------------- Helper Functions ------------------------------*/

void
ROSFactoryAllocation::
ParseTaskPoint(XMLNode& _node, ros::NodeHandle& _nh) {

  TaskPoint tp;
  tp.label = _node.Read("label",true,"","Label for task point.");
  tp.topic = _node.Read("topic",true,"","Topic to track countdown.");
  tp.threshold = _node.Read("threshold",false,20.,0.,MAX_DBL,
      "Threshold to add to allocation.");
  tp.priority = _node.Read("priority",false,tp.priority,size_t(0),MAX_UINT,
      "Priority value of task point.");


  std::string point = _node.Read("location",true,"","Location of the task point.");
  std::istringstream buffer(point);

  double d;
  for(size_t i = 0; i < 3; i++) {
    buffer >> d;
    tp.location.push_back(d);
  }

  m_taskPoints[tp.label] = tp;

  //m_countdownSubs[tp.label] = _nh.subscribe<std_msgs::Float32>(tp.topic, 1, 
  //    boost::bind(&ROSFactoryAllocation::TimeRemainingCallback,this,_1,tp.label));
  //m_countdownSubs[tp.label] = _nh.subscribe(tp.topic, 1000, 
  //    TimeRemainingCallback);
}

void
ROSFactoryAllocation::
UpdateTimeRemaining(std::string _label) {

  auto topic = _label + "/" + m_taskPoints[_label].topic;
  auto sharedMsg = ros::topic::waitForMessage<std_msgs::Float32>(topic,ros::Duration(5));

  if(sharedMsg == NULL) {
    return;
  }

  auto msg = *sharedMsg;

  if(msg.data > m_countdown[_label]) {
    m_allocated.erase(_label);
  }

  m_countdown[_label] = msg.data;
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
BuildDecomposition(std::vector<std::string> _labels) {

  if(_labels.empty())
    return std::unique_ptr<Decomposition>(nullptr);

  std::cout << "Building decomposition for: " << _labels << std::endl;

  std::shared_ptr<SemanticTask> top(new SemanticTask());
  std::unique_ptr<Decomposition> decomp(new Decomposition(top));

  auto robot = this->m_agent->GetRobot();

  for(auto label : _labels) {
    auto tp = m_taskPoints[label];

    std::shared_ptr<MPTask> mpTask(new MPTask(robot));
    Cfg cfg(robot);
    cfg[0] = m_depot[0];
    cfg[1] = m_depot[1];

    std::unique_ptr<CSpaceConstraint> startConstraint(new CSpaceConstraint(robot,cfg));
    mpTask->SetStartConstraint(std::move(startConstraint));

    auto loc = tp.location;
    cfg[0] = loc[0];
    cfg[1] = loc[1];

    std::unique_ptr<CSpaceConstraint> goalConstraint(new CSpaceConstraint(robot,cfg));
    mpTask->AddGoalConstraint(std::move(goalConstraint));

    std::shared_ptr<SemanticTask> st(new SemanticTask(
          label,top.get(),decomp.get(),mpTask,false));
    decomp->AddTask(st);
  }

  return decomp;
}

void
ROSFactoryAllocation::
PlanAllocations(Decomposition* _decomp) {

  if(m_planning)
    return;

  auto c = dynamic_cast<Coordinator*>(this->m_agent);
  auto prob = c->GetRobot()->GetMPProblem();
  auto lib = c->GetTMPLibrary();

  std::vector<Robot*> team;
  for(auto agent : c->GetChildAgents()) {
    team.push_back(agent->GetRobot());
  }

  m_plan->SetTeam(team);
  m_plan->SetDecomposition(_decomp);
  
  m_planning = true;
  lib->Solve(prob,_decomp,m_plan.get(),c,team);
  DistributeTasks();
  m_planning = false;
}

void
ROSFactoryAllocation::
DistributeTasks() {
  auto c = dynamic_cast<Coordinator*>(this->m_agent);

  for(auto agent : c->GetChildAgents()) {
    auto robot = agent->GetRobot();

    auto allocs = m_plan->GetAllocations(robot);
    std::cout << "ALOCATION for " << robot->GetLabel() << ":" << std::endl;
    for(auto alloc : allocs) {
      std::cout << "\t" << alloc->GetLabel() << std::endl;
    }
    for(auto alloc : allocs) {

      auto sol = m_plan->GetTaskSolution(alloc);
      auto path = sol->GetPath();
      const auto& cfgs = path->Cfgs();
      
      auto start = cfgs[0];
      AssignTask(robot->GetLabel(),"depot",start.GetData());
      auto goal = cfgs.back();
      AssignTask(robot->GetLabel(),alloc->GetLabel(),goal.GetData());
    }
  }

}

/*------------------------------ ROS Interface -------------------------------*/

/*----------------------------------------------------------------------------*/
