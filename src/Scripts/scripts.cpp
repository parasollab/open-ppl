#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <cstdlib>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv){
 
  if(argc != 11){
    cerr << "Error::Invalid Usage" << endl
      << "Usage: ./scripts -e MyEnvironment -x pmpl_MyPMPL -n 10 -q parallel -t tests" << endl
      << "Where:" << endl
      << "\t-e - Environment" << endl
      << "\t-x - PMPL Executable" << endl
      << "\t-n - Number of tests" << endl
      << "\t-q - Bigspring queue" << endl
      << "\t-t - Test file with space separated list of tests" << endl;
    exit(1);
  }

  string env, executable, queue, testfile;
  int num;

  //read options from command line
  //-e - the environment name, e.g., Maze
  //-x - the executable name for the jobs, e.g., pmpl
  //-n - the number of jobs you want to run, e.g., 10
  //-q - the queue on the machine you will run on (parallel, large, medium, or
  //small)
  //-t - a filename of the set of test methods. Space separated list in
  //external file.
  //
  //n job files are created per test to run the executable on the appropriate
  //queue. Naming scheme for these files is env.test.#.sh. Also created are two
  //sets of master scripts. There is one master script per test to submit all 
  //jobs associated with it. There is also one master script to submit all jobs.
  //
  //Assumption: For each job there is a file env.test.#.xml which will be the
  //input to the pmpl executable
  
  char c;
  while((c = getopt(argc, argv, "e:x:n:q:t:")) != -1){
    switch(c){
      case 'e':
        env = optarg;
        break;
      case 'x':
        executable = optarg;
        break;
      case 'n':
        num = atoi(optarg);
        break;
      case 'q':
        queue = optarg;
        break;
      case 't':
        testfile = optarg;
        break;
      case '?':
        if(optopt == 'e' || optopt == 'x' || optopt == 'n'
            || optopt == 'q' || optopt == 't')
          cerr << "Option -" << optopt << " requires an argument." << endl;
        else if(isprint(optopt))
          cerr << "Unknown option `-" << c << "'." << endl;
        else
          cerr << "Unknwon option character `" << optopt << "'" << endl;
        exit(1);
      default:
        cerr << "Who knows what just happened?" << endl;
        exit(1);
    }
  }

  cerr << "Creating scripts with values" << endl << endl
    << "executable:\t" << executable << endl
    << "environment:\t" << env << endl
    << "queue:\t" << queue << endl
    << "num:\t" << num << endl
    << "tests:\t" << testfile << endl << endl;

  //read the test file
  vector<string> tests;
  string test="";
  ifstream iftests(testfile.c_str());
  while(iftests >> test){
    cout<<"Test:\t"<<test<<endl;
    tests.push_back(test);
  }
 
  //generate the job files
  {//force the scope to automatically close the ofstreams
    typedef vector<string>::iterator SIT;
    
    map<string, ofstream*> testscripts; //set of master script ofstreams per test
    
    //initialize the master script ofstreams
    for(SIT sit = tests.begin(); sit!=tests.end(); ++sit){
      string testsh = env+"."+*sit+".sh";
      testscripts[*sit] = new ofstream(testsh.c_str());
    }

    //output job files
    for(int i = 0; i<num; i++){
      //create the ending of the file name i.sh
      ostringstream oss;
      oss<<i;

      //output the test file for job i
      for(SIT sit = tests.begin(); sit!=tests.end(); ++sit){
        string job = env+"."+*sit+"."+oss.str();
        string jobfile = job+".sh";

        ofstream ofjob(jobfile.c_str());
        ofjob<<"#!/bin/bash\n#PBS"
          << " -q " << queue 
          << " -l mem=1000mb"
          << " -l walltime=20:00:00"
          << " -N " << job
          << " -V" << endl
          << "cd ./" << env << "/" << endl
          << "../" << executable << " -f " << job << ".xml >& " << job << ".log" << endl;

        (*testscripts[*sit]) << "qsub " << jobfile << endl;
      }
    }

    //add test master scripts to overall master script
    string envsh = env+".sh"; //master script string name
    ofstream scripts(envsh.c_str()); //master script ofstream
    for(SIT sit = tests.begin(); sit!=tests.end(); sit++){
      string testsh = env+"."+*sit+".sh";
      scripts << "./" << testsh << endl;
    }
  }
  system("chmod 777 *.sh");
}
