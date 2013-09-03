#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv){
  if(argc != 11){
    cerr << "Error::Invalid Usage" << endl
      << "Usage: ./xmlgen -e MyEnvironment -x MyXML.xml -n 10 -t tests -s 123" << endl
      << "Where:" << endl
      << "\t-e - Environment" << endl
      << "\t-x - XML file" << endl
      << "\t-n - Number of tests" << endl
      << "\t-t - Test file with space separated list of tests" << endl
      << "\t-s - Random seed to generate random seeds" << endl;
    exit(1);
  }

  string env, xmlfile, testfile;
  int num, seed;

  //read options from command line
  //-e - the environment name, e.g., Maze
  //-x - the template xml file, e.g., PMPLExamples.xml
  //-n - the number of jobs you want to run, e.g., 10
  //-t - a filename of the set of test methods. Space separated list in
  //external file.
  //-s - Random seed to generate n random seeds for test files
  //
  //n xml files are created per test, where the solver and random seed is set 
  //appropriately. Naming scheme for these files is env.test.#.xml.
  //
  //Assumptions: In the master XML file - the random seed on the solver is set
  //to 0 and the mpStrategyMethod is the first name that appears in the
  //testsfile.
  
  char c;
  while((c = getopt(argc, argv, "e:x:n:t:s:")) != -1){
    switch(c){
      case 'e':
        env = optarg;
        break;
      case 'x':
        xmlfile = optarg;
        break;
      case 'n':
        num = atoi(optarg);
        break;
      case 't':
        testfile = optarg;
        break;
      case 's':
        seed = atoi(optarg);
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
    << "executable:\t" << xmlfile << endl
    << "environment:\t" << env << endl
    << "num:\t" << num << endl
    << "tests:\t" << testfile << endl
    << "seed:\t" << seed << endl << endl;

  //read the test file
  vector<string> tests;
  string test="";
  ifstream iftests(testfile.c_str());
  while(iftests >> test){
    cout<<"Test:\t"<<test<<endl;
    tests.push_back(test);
  }

  srand(seed);

  for(int i=0; i<num; i++){
    ostringstream oss;
    oss << i;
    string ival = oss.str();

    oss.str("");
    oss << rand()%999999999;
    string rseed = oss.str();

    typedef vector<string>::iterator SIT;
    for(SIT sit = tests.begin(); sit!=tests.end(); ++sit){
      //set filename
      string copyxmlfile = env + "." + *sit + "." + ival + ".xml";

      //copy to new file
      string cmd = "cp " + xmlfile + " " + copyxmlfile;
      system(cmd.c_str());

      //set random seed
      cmd = "sed -i 's/seed=\"0\"/seed=\"" + rseed + "\"/' " + copyxmlfile;
      system(cmd.c_str());

      //make appropriate for test
      cmd = "sed -i 's/mpStrategyLabel=\"" + tests[0] + 
        "\"/mpStrategyLabel=\"" + *sit + "\"/' " + copyxmlfile;
      system(cmd.c_str());
      
      //make appropriate for test output
      cmd = "sed -i 's/baseFilename=\"" + tests[0] + "\\/" + tests[0] +
        "\"/baseFilename=\"" + *sit + "\\/" + *sit + "." + ival + "\"/' " + copyxmlfile;
      system(cmd.c_str());
    }
  }
}
