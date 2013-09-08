#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>

using namespace std;

int main(int argc, char** argv){
  int filterNum = atoi(argv[1]);
  for(int i = 2; i<argc; i++){
    string filename = argv[i];
    string cmd = "grep \"Number of Nodes\" " + filename + " > filter.out"; 
    system(cmd.c_str());

    ifstream ifs("filter.out");
    string tmp;
    ifs >> tmp;
    ifs >> tmp;
    ifs >> tmp;
    ifs >> tmp;
    int dtmp = atoi(tmp.c_str());
    //cout << filterNum << "\t" << dtmp;
    if(dtmp < filterNum){
      cout << filename << endl;
    }
  }
  system("rm filter.out");
}
