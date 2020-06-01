#include <exception>
#include <limits>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netinet/in.h>
#include <netdb.h>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "Utilities/PMPLExceptions.h"

#include "Communication/Communicator.h"

void error(char* msg) {
	perror(msg);
	exit(1);
}

std::string GetMessage(int _newsockfd) {
	char buffer[256];
	int n;
  bzero(buffer,256);
  n = read(_newsockfd,buffer,255);
  if (n < 0) 
  	throw RunTimeException(WHERE) << "ERROR reading from socket";
  printf("Here is the message: %s\n",buffer);
  if (n < 0) 
	  throw RunTimeException(WHERE) << "ERROR writing to socket";

	//TODO::Convert to MPTask pointer
	std::string message;
	for(auto c : buffer) {
		if(c == '\0')
			break;
		message += c;
	}
	return message;
}

CSpaceConstraint* ReceiveCSpaceConstraint(int _newsockfd, Robot* _robot) {
	std::cout << "Receving CSpace Constraint" << std::endl;
	std::string message = GetMessage(_newsockfd);
	
	if(write(_newsockfd,"Message received.",18) < 0)
  	throw RunTimeException(WHERE) << "ERROR writing to socket";

	std::stringstream ss(message);
	std::vector<double> dofs;
	std::cout << "dofs: ";
	while(true) {
		double dof;
		ss >> dof;
		if(!ss)
			break;
		dofs.push_back(dof);
		std::cout << dof << " ";
	}
	std::cout << endl;
	//Add Check that # of dofs match robot

	Cfg cfg(_robot);
	cfg.SetData(dofs);

	CSpaceConstraint* constraint = new CSpaceConstraint(_robot,cfg);
	
	return constraint;
}

bool ReceiveStartConstraint(int _newsockfd, MPTask* _task) {
	std::cout << "Receving Start Constraint" << std::endl;
	std::string message = GetMessage(_newsockfd);
	int startPos = message.rfind("start=",0);
	std::cout << "startPos: " << startPos << std::endl;
	if(startPos != 0){
		int n = write(_newsockfd,"Message should start with start constraint type (start=<constraint-type>)",76);
  	if (n < 0) 
	  	throw RunTimeException(WHERE) << "ERROR writing to socket";
		return false;
	}

	if(write(_newsockfd,"Message received.",18) < 0)
  	throw RunTimeException(WHERE) << "ERROR writing to socket";

	std::string constraintType = message.substr(6,-1);
	std::cout << "Start Constraint Type: " << constraintType << std::endl;
	
	Constraint* constraint;
	if(constraintType == "cspace"){
		constraint = ReceiveCSpaceConstraint(_newsockfd, _task->GetRobot());
	}	

	_task->SetStartConstraint(std::move(std::unique_ptr<Constraint>(constraint)));

	return true;
}

bool ReceiveGoalConstraint(int _newsockfd, MPTask* _task) {
	std::cout << "Receving Goal Constraint" << std::endl;
	std::string message = GetMessage(_newsockfd);
	int goalPos = message.rfind("goal=",0);
	std::cout << "goalPos: " << goalPos << std::endl;
	if(goalPos != 0){
		int n = write(_newsockfd,"Message should goal with goal constraint type (goal=<constraint-type>)",73);
  	if (n < 0) 
	  	throw RunTimeException(WHERE) << "ERROR writing to socket";
		return false;
	}
	if(write(_newsockfd,"Message received.",18) < 0)
  	throw RunTimeException(WHERE) << "ERROR writing to socket";

	std::string constraintType = message.substr(5,-1);
	std::cout << "Goal Constraint Type: " << constraintType << std::endl;
	
	Constraint* constraint;
	if(constraintType == "cspace"){
		constraint = ReceiveCSpaceConstraint(_newsockfd, _task->GetRobot());
	}	

	_task->AddGoalConstraint(std::move(std::unique_ptr<Constraint>(constraint)));

	return true;
}

MPTask* ReceiveTask(int _newsockfd, MPProblem* _problem) {
	std::cout << "Receiving task" << std::endl;

	std::string message = GetMessage(_newsockfd);

	int len = message.size();
	int labelPos = message.rfind("label=",0);
	std::cout << "LabelPos: " << labelPos << std::endl;
	if(labelPos != 0){
		int n = write(_newsockfd,"Message should start with task label (label=<task-label>)",57);
  	if (n < 0) 
	  	throw RunTimeException(WHERE) << "ERROR writing to socket";
		return nullptr;
	}
	int space = message.find(" ");
	std::cout << "SPACE INDEX: " << space << std::endl;
	std::string label = message.substr(6,space-6);
	std::cout << "Label: " << label << "\n" << std::endl;

	//Find the robot label	
	int robotPos = message.find("robot=");
	if(robotPos > len or robotPos == -1){
		int n = write(_newsockfd,"Message should contain a robot label (robot=<robot-label>)",59);
  	if (n < 0) 
	  	throw RunTimeException(WHERE) << "ERROR writing to socket";
		return nullptr;
	}
	else {
  	int n = write(_newsockfd,"Received labels. Send Start Constraint next.",44);
  	if (n < 0) 
	  	throw RunTimeException(WHERE) << "ERROR writing to socket";
	}
	space = message.find(" ",space+1);
	std::cout << "SPACE INDEX: " << space << std::endl;
	std::string robotLabel = message.substr(robotPos+6,len-(robotPos+7));
	std::cout << "RobotLabel: " << robotLabel << "\n" << std::endl;	

	MPTask* newTask = new MPTask(_problem->GetRobot(robotLabel));
	newTask->SetLabel(label);

	// Receive Start Constraint
	
	if(!ReceiveStartConstraint(_newsockfd, newTask) or !ReceiveGoalConstraint(_newsockfd, newTask))
		return false;

	return newTask;
}

void SendCfg(int _newsockfd,Cfg _cfg) {

	std::string message = GetMessage(_newsockfd);
	std::cout << message << std::endl;

	std::cout << "Sending Cfg: " << _cfg.PrettyPrint() << std::endl;

	std::string dofs = "cfg ";
	for(size_t i = 0; i < _cfg.GetData().size(); i++) {
		auto dof = _cfg.GetData()[i];
		if(i > 0)
			dofs += " ";
		auto s = to_string(dof);
		dofs += s;
	}
	std::cout << "Writing dofs: " << dofs << std::endl;
	char msg[dofs.size()+1];
	strcpy(msg,dofs.c_str());
	if(write(_newsockfd,msg,dofs.size()) < 0)
		throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;
}

int
main(int _argc, char** _argv) {
	std::cout << "Running server" << std::endl;
  // Assert that this platform supports an infinity for doubles.
  if(!std::numeric_limits<double>::has_infinity)
    throw RunTimeException(WHERE) << "This platform does not support infinity "
                                  << "for double-types, which is required for "
                                  << "pmpl to work properly.";

  if(_argc != 4 || std::string(_argv[1]) != "-f")
    throw ParseException(WHERE) << "Incorrect usage. Usage: -f options.xml";

  // Get the XML file name from the command line.
  std::string xmlFile = _argv[2];

  // Parse the Problem node into an MPProblem object.
  MPProblem* problem = new MPProblem(xmlFile);

  // Parse the Library node into an MPLibrary object.
  MPLibrary* pmpl = new MPLibrary(xmlFile);

	Communicator comm(8888,8889);

	comm.RegisterWithMaster(8888,"localhost");

	comm.CreatePublisher("test");
	comm.Listen();










	// Connect to client
	int sockfd, newsockfd, portno;
  socklen_t clilen;
  char buffer[256];
  struct sockaddr_in serv_addr, cli_addr;
  int n;
  //if (_argc < 2) {
  //    fprintf(stderr,"ERROR, no port provided\n");
  //    exit(1);
  //}
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0) 
  	throw RunTimeException(WHERE) << "ERROR opening socket" << std::endl;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  portno = atoi(_argv[3]);
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(portno);
  if(bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
    throw RunTimeException(WHERE) << "ERROR on binding" << std::endl;
  listen(sockfd,5);
  clilen = sizeof(cli_addr);
  newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
  if(newsockfd < 0) 
    throw RunTimeException(WHERE) << "ERROR on accept" << std::endl;

	// Wait for messages
	while(true) {
    bzero(buffer,256);
    n = read(newsockfd,buffer,255);
    if (n < 0) 
  		throw RunTimeException(WHERE) << "ERROR reading from socket" << std::endl;
    printf("Here is the message: %s\n",buffer);
    n = write(newsockfd,"I got your message",18);
    if (n < 0) 
		throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;

		std::string message;
		for(auto c : buffer) {
			if(c == '\0')
				break;
			message += c;
		}
		std::cout << "Message: " << message << std::endl;
		if(message == "task\n" or message == "task") {
			MPTask* newTask = ReceiveTask(newsockfd, problem);
			MPSolution* solution = new MPSolution(newTask->GetRobot());
      pmpl->Solve(problem, newTask, solution);
			std::cout << "Solution: " << solution << std::endl;
			for(auto cfg : solution->GetPath(newTask->GetRobot())->Cfgs()) {
				SendCfg(newsockfd,cfg);
			}
			n = write(newsockfd,"done",4);
		}
		else {
			std::cout << message.size(); 
			std::cout << message[0]; 
			std::cout << message[4]; 
		}
	}



  // Create storage for the solution and ask the library to solve our problem.
  Robot* const robot = problem->GetRobots().front().get();
  const auto robotTasks = problem->GetTasks(robot);
    for(auto task : robotTasks)
      if(!task->GetStatus().is_complete())
      	pmpl->Solve(problem, task.get());

  // Also solve the group task(s).
  if(!problem->GetRobotGroups().empty()) {
    RobotGroup* const robotGroup = problem->GetRobotGroups().front().get();
    for(auto groupTask : problem->GetTasks(robotGroup))
      pmpl->Solve(problem, groupTask.get());
  }

  if(robotTasks.empty() and (problem->GetRobotGroups().empty() or
      problem->GetTasks(problem->GetRobotGroups().front().get()).empty()))
    throw RunTimeException(WHERE) << "No tasks were specified!";

  // Release resources.
  delete problem;
  delete pmpl;

  close(newsockfd);
  close(sockfd);
  return 0;
}
