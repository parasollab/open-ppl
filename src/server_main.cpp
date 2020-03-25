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

void error(char* msg) {
	perror(msg);
	exit(1);
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
     if (sockfd < 0) 
        throw RunTimeException(WHERE) << "ERROR opening socket";
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(_argv[3]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              throw RunTimeException(WHERE) << "ERROR on binding";
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          throw RunTimeException(WHERE) << "ERROR on accept";
     bzero(buffer,256);
     n = read(newsockfd,buffer,255);
     if (n < 0) 
			throw RunTimeException(WHERE) << "ERROR reading from socket";
     printf("Here is the message: %s\n",buffer);
     n = write(newsockfd,"I got your message",18);
     if (n < 0) 
			throw RunTimeException(WHERE) << "ERROR writing to socket";
     close(newsockfd);
     close(sockfd);
     return 0; 


  // Get the XML file name from the command line.
  std::string xmlFile = _argv[2];

  // Parse the Problem node into an MPProblem object.
  MPProblem* problem = new MPProblem(xmlFile);

  // Parse the Library node into an MPLibrary object.
  MPLibrary* pmpl = new MPLibrary(xmlFile);

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

  return 0;
}
