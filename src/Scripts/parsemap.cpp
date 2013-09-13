#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>

using namespace std;
int main(int argc, char*argv[])
{
  for (int i = 1; i < argc; ++i) {
    char buffer[300];
    ifstream fin(argv[i], ios::in);
    ofstream fout((string(argv[i]) + ".coords").c_str(), ios::out);
    if( !fin.good() )
    {	//Not good. File not found
      cerr<<"File  not found"<<endl;
      return false;
    }
    if( !fout.good() )
    {	//Not good. File not found
      cerr<<"File  not found"<<endl;
      return false;
    }
    string temp;
    bool flag = false;
    double x, y,z; 
    int m, n;
    int num;
    while(!fin.eof())
    {
      fin>>temp;
      if(temp.find("GRAPHSTART") != -1){
        flag = true;
        fin>>num;
        fout<<num<<endl;
        fin.getline(buffer,500);
        for(int i=0;i<num;++i){
          fin.getline(buffer,500);
          stringstream str(buffer);
          str>>m>>n>>x>>y>>z;  
          fout<<x<<"\t"<<y<<"\t"<<z<<endl;
        }
      }

    }

    fin.close();
    fout.close();

  }
}
