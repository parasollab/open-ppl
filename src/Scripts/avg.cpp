#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

using namespace std;

void Analyze(vector<double>& _v);

int main(int argc, char** argv) {
  string name = argv[1];
  int step = atoi(argv[2]);
  int offset = atoi(argv[3]);
  
  typedef vector<double>::iterator DIT;
  vector<double> data;
  
  // get data
  double dtmp;
  for(int i = 4; i < argc; i += step) {
    istringstream iss(argv[i+step-offset-1]);
    iss>>dtmp;
    data.push_back(dtmp);
  }

  cout << name;

  Analyze(data);

  // sort, find quartiles
  sort(data.begin(), data.end());
  int num = data.size();

  double q1, q3;
  int mid1, mid2;
  if(num%2 == 1){ //has median case
    //cout << "\nThere is a median::";
    mid1 = num/2 - 1;
    mid2 = num/2 + 1;
    //cout << "\tmid1::" << mid1 << "\tmid2::" << mid2 << endl;
  }
  else{
    //cout << "\nThere is no median::";
    mid1 = num/2-1;
    mid2 = num/2;
    //cout << "\tmid1::" << mid1 << "\tmid2::" << mid2 << endl;
  }

  if((mid1+1)%2 == 1){ //has median
    //cout << "\nq1 is at::" << mid1/2 << endl;
    q1 = data[mid1/2];
  }
  else{
    //cout << "\nq1 is between::" << mid1/2 << " and " << mid1/2+1 << endl;
    q1 = (data[mid1/2] + data[mid1/2+1])/2;
  }

  if((num+mid2)%2 == 1){ //has median
    //cout << "\nq3 is at::" << (num+mid2-1)/2 << endl;
    q3 = data[(num+mid2-1)/2];
  }
  else{
    //cout << "\nq3 is between::" << (num+mid2-1)/2 << " " << (num+mid2-1)/2+1 << endl;
    q3 = (data[(num+mid2-1)/2] + data[(num+mid2-1)/2+1])/2;
  }


  //cout << "\tq1=" << q1 << "\tq3=" << q3;

  if(data.size() == 1)
    q1 = q3 = data[0];

  // delete outliers
  double iqr = q3-q1;
  for(DIT dit = data.begin(); dit!=data.end();){
    if(*dit < q1-1.5*iqr || *dit > q3+1.5*iqr)
      dit = data.erase(dit);
    else
      ++dit;
  }

  Analyze(data);
  
  cout << endl;

  return 0;
}

void Analyze(vector<double>& _v){
  //cout << "\nList::" << endl;
  // get new count, get avg
  double num = _v.size();
  double sum = accumulate(_v.begin(), _v.end(), 0.0);
  double avg = sum/num;

  // get std dev
  double std;
  typedef vector<double>::iterator DIT;
  for(DIT dit = _v.begin(); dit != _v.end(); ++dit){
    //cout << *dit << " " << endl;
    std+=(*dit-avg)*(*dit-avg);
  }
  if(_v.size() == 1)
    std = 0;
  else
    std = sqrt(std/num);
  
  cout << "\t" << num << "\t" << avg << "\t" << std;
}
