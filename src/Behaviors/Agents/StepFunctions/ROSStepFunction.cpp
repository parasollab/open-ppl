#include "ROSStepFunction.h"

#include "MPProblem/Robot/Robot.h"

/*----------------------- Construction --------------------*/
ROSStepFunction::
ROSStepFunction(Agent* _agent, XMLNode& _node) 
              : StepFunction(_agent,_node) {

  int argc = 0;
  char* argv[255];

  ros::init(argc,argv,this->m_agent->GetRobot()->GetLabel());

  ros::start();
  
  ROS_INFO_STREAM("Hello World!");

  /*
  ros::spin();

  ros::shutdown();
  */
}

ROSStepFunction::
~ROSStepFunction() { }

/*------------------------- Interface -----------------------*/

void 
ROSStepFunction::
StepAgent(double _dt) {
  std::cout << "Calling step function." << std::endl;
}
