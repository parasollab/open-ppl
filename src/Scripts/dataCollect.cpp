#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <cstdlib>

using namespace std;

typedef map<string, pair<double, double> > Data;
typedef pair<string, Data> MethodData;
typedef vector<MethodData> EnvironmentData;
typedef vector<EnvironmentData> AllData;

void PrintData(string filename, string ext, vector<EnvironmentData>& ved,
    vector<string>& envs, vector<string>& methods, vector<string>& metrics);

int main(int argc, char** argv){
 
  if(argc < 4){
    cerr << "Error::Incorrect Usage." << endl
      << "Usage: ./dataCollect expName n m files" << endl
      << "Where:" << endl
      << "\texpName - output base name" << endl
      << "\tn - number of methods experimented with" << endl
      << "\tm - number of metrics collected" << endl
      << "\tfiles - output files from the program gatherData" << endl;
    exit(1);
  }

  string basename = argv[1];
  int numMethods = atoi(argv[2]), 
      numMetrics = atoi(argv[3]);

  AllData all, allNoOut;
  vector<string> environments, methods, metrics;

  for(int i = 4; i<argc; i++){
    EnvironmentData e, eNoOut;
    ifstream ifs(argv[i]);

    string environment;
    ifs >> environment;
    environments.push_back(environment);

    while(ifs){
      MethodData me, meNoOut;
      
      string method;
      ifs >> method;
      me.first = method;
      me.second = Data();
      if(methods.size() < numMethods)
        methods.push_back(method);

      for(int i = 0; i < numMetrics; i++){
        string metric;
        int n, n2; 
        double m, s, m2, s2;
        ifs >> metric >> n >> m >> s >> n2 >> m2 >> s2;

        if(metrics.size() < numMetrics)
          metrics.push_back(metric);
        
        me.second[metric] = make_pair(m, s);
        meNoOut.second[metric] = make_pair(m2, s2);

      }

      e.push_back(me);
      eNoOut.push_back(meNoOut);
    }

    all.push_back(e);
    allNoOut.push_back(eNoOut);
  }

  PrintData(basename, "dat", all, environments, methods, metrics);
  PrintData(basename, "noout.dat", allNoOut, environments, methods, metrics);
  
  return 0;
}

void PrintData(string basename, string ext, vector<EnvironmentData>& ved,
    vector<string>& envs, vector<string>& methods, vector<string>& metrics){
  for(int i = 0; i<metrics.size(); i++){
    string name = basename+"."+metrics[i]+"."+ext;
    ofstream ofs(name.c_str());

    ofs << "Environment ";
    for(int j = 0; j<methods.size(); j++){
      ofs << methods[j] << " " << methods[j] << " ";
    }
    ofs << endl;

    for(int j = 0; j<envs.size(); j++){
      ofs << envs[j] << " ";
      for(int k = 0; k<methods.size(); k++){
        ofs << ved[j][k].second[metrics[i]].first << " " 
          << ved[j][k].second[metrics[i]].second << " ";
      }
      ofs << endl;
    }
  }
}
