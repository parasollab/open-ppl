#include <exception>
#include <limits>
#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "MPLibrary/PMPL.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "Utilities/PMPLExceptions.h"

#include "Communication/Communicator.h"

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

std::string GetMessage(int _newsockfd) {
	char buffer[1024];
	int n;
  bzero(buffer,1024);
  n = read(_newsockfd,buffer,1023);
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

void SendLabels(int _sockfd, std::string _task, std::string _robot) {
	std::string message = "label=" + _task + " robot=" + _robot + " ";
	int len = message.size();
	char msg[len+1];
	strcpy(msg,message.c_str());
  int n = write(_sockfd,msg,len);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;
	char buffer[256];
  bzero(buffer,256);
  n = read(_sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket" << std::endl;
  printf("%s\n",buffer);
}

void SendConstraints(int _sockfd, std::string _start, std::string _goal) {
	std::string message = "start=cspace";
	int len = message.size();
	char msg[len+1];
	strcpy(msg,message.c_str());
  int n = write(_sockfd,msg,len);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket";
	char buffer[256];
  bzero(buffer,256);
  n = read(_sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket";
  printf("%s\n",buffer);

	std::cout << "sending start constraint: " << _start << std::endl;

	len = _start.size();
	char msg2[len+1];
	strcpy(msg2,_start.c_str());
  n = write(_sockfd,msg2,len);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;
  bzero(buffer,256);
  n = read(_sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket" << std::endl;
  printf("%s\n",buffer);

	message = "goal=cspace";

	len = message.size();
	char msg3[len+1];
	strcpy(msg3,message.c_str());
  n = write(_sockfd,msg3,len);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;
  bzero(buffer,256);
  n = read(_sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket" << std::endl;
  printf("%s\n",buffer);

	std::cout << "sending goal constraint: " << _goal << std::endl;
	len = _goal.size();
	char msg4[len+1];
	strcpy(msg4,_goal.c_str());
  n = write(_sockfd,msg4,len);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket" << std::endl;
  bzero(buffer,256);
  n = read(_sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket" << std::endl;
  printf("%s\n",buffer);
}

Cfg ReceiveCfg(std::string _msg, Robot* _robot) {
	std::cout << "Parsing cfg: " << _msg << std::endl;
	stringstream ss(_msg);
	std::vector<double> dofs;
	while(true) {
		double dof;
		ss >> dof;
		if(!ss) 
			break;
		dofs.push_back(dof);
	}
	std::cout << "DOFS: " << dofs.size() << std::endl;
	Cfg cfg(_robot);
	cfg.SetData(dofs);
	return cfg;
}

std::vector<Cfg> ReceivePlan(int _sockfd, Robot* _robot) {
	std::cout << "Receving Plan" << std::endl;

	std::vector<Cfg> plan = {};

	while(true) {
		if(write(_sockfd,"Cfg Please",10) < 0)
			throw RunTimeException(WHERE) << "ERROR writing to socket." << std::endl;

		std::cout << "Waiting on cfg" << std::endl;
		auto message = GetMessage(_sockfd);

		if(message.rfind("cfg",0) == 0){
			auto cfgString = message.substr(4,-1);
			auto cfg = ReceiveCfg(cfgString,_robot);
			std::cout << cfg.PrettyPrint() << std::endl;
			plan.push_back(cfg);
		}
		else if(message.rfind("done",0) == 0) {
			return plan;
		}

	}
}

int
main(int _argc, char** _argv) {
	std::cout << "Running client" << std::endl;
  // Assert that this platform supports an infinity for doubles.
  if(!std::numeric_limits<double>::has_infinity)
    throw RunTimeException(WHERE) << "This platform does not support infinity "
                                  << "for double-types, which is required for "
                                  << "pmpl to work properly.";

	Communicator comm(8888,8890);

	comm.RegisterWithMaster(8888,"localhost");

	comm.CreateSubscriber("test");
	//comm.Listen();








  if(_argc != 5 || std::string(_argv[1]) != "-f")
    throw ParseException(WHERE) << "Incorrect usage. Usage: -f options.xml";

	std::string xmlFile = _argv[2];
	MPProblem* problem = new MPProblem(xmlFile);

	int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    char buffer[256];
    portno = atoi(_argv[4]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        throw RunTimeException(WHERE) << "ERROR opening socket";
    server = gethostbyname(_argv[3]);
    if (server == NULL) {
        throw RunTimeException(WHERE) << "ERROR, no such host\n";
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        throw RunTimeException(WHERE) << "ERROR connecting";

	
  n = write(sockfd,"task",4);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR writing to socket";
  bzero(buffer,256);
  n = read(sockfd,buffer,255);
  if (n < 0) 
    throw RunTimeException(WHERE) << "ERROR reading from socket";
  printf("%s\n",buffer);

	auto robot = problem->GetRobots().front().get();
	auto robotTasks = problem->GetTasks(robot);
	auto task = robotTasks[0];

	std::string taskLabel = task->GetLabel();
	std::string robotLabel = robot->GetLabel();
	SendLabels(sockfd, taskLabel, robotLabel);

	
	//Assume CSpace point constraint  and only one goal constraint for now.
	auto boundary = task->GetStartConstraint()->GetBoundary()->GetCenter();
	std::string startConstraint;
	for(size_t i = 0; i < boundary.size(); i++) {
		auto dof = boundary[i];
		if(i > 0)
			startConstraint += " ";
		startConstraint += to_string(dof);
	}

	boundary = task->GetGoalConstraints()[0]->GetBoundary()->GetCenter();
	std::string goalConstraint;
	for(size_t i = 0; i < boundary.size(); i++) {
		auto dof = boundary[i];
		if(i > 0)
			goalConstraint += " ";
		goalConstraint += to_string(dof);
	}


	SendConstraints(sockfd, startConstraint, goalConstraint);

	auto plan = ReceivePlan(sockfd,robot);

	std::cout << "Outputting plan: " << std::endl;
	for(auto cfg : plan) {
		std::cout << cfg.PrettyPrint() << std::endl;
	}
	std::cout << std::endl << std::endl;

	while(true) {
    printf("Please enter the message: ");
    bzero(buffer,256);
    fgets(buffer,255,stdin);
    n = write(sockfd,buffer,strlen(buffer));
    if (n < 0) 
         throw RunTimeException(WHERE) << "ERROR writing to socket";
    bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         throw RunTimeException(WHERE) << "ERROR reading from socket";
    printf("%s\n",buffer);
	}
    close(sockfd);
    return 0;
}
