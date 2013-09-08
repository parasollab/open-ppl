#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>

using namespace std;

int main(int argv, char** argc){
  if(argv < 4){
    cerr << "Must provide normalized <filename> <index> <outfilename>" << endl;
    exit(1);
  }

  string filename = argc[1];
  string outfilename = argc[3];
  double index = atoi(argc[2]);

  ifstream ifs(filename.c_str());
  ofstream ofs(outfilename.c_str());
  string tmp;
  //read first line of input
  getline(ifs, tmp);
  ofs << tmp << endl;
  while(getline(ifs, tmp)){
    istringstream iss(tmp);
    string tmp2;
    iss >> tmp2;
    ofs << tmp2 << "\t";
    double dtmp;
    vector<double> data;
    while(iss >> dtmp){
      data.push_back(dtmp);
    }
    for(size_t i = 0; i<data.size(); i++){
      ofs << data[i]/data[index] << "\t";
    }
    ofs << endl;
  }
  ifs.close();
  ofs.close();
  return 0;
}
