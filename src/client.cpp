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

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int
main(int _argc, char** _argv) {
	std::cout << "Running client" << std::endl;
  // Assert that this platform supports an infinity for doubles.
  if(!std::numeric_limits<double>::has_infinity)
    throw RunTimeException(WHERE) << "This platform does not support infinity "
                                  << "for double-types, which is required for "
                                  << "pmpl to work properly.";

  if(_argc != 5 || std::string(_argv[1]) != "-f")
    throw ParseException(WHERE) << "Incorrect usage. Usage: -f options.xml";


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
    close(sockfd);
    return 0;
}
