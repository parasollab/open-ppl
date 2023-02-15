/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <map>
#include <time.h>
#include <sys/time.h>
#include <stapl/runtime.hpp>

using namespace std;

//////////////////////////////////////////////////////////////////////
// set the seed based on the time
//////////////////////////////////////////////////////////////////////

void set_random_seed(void)
{
    time_t timer, temp;
    temp = time(&timer);
    if (temp == (time_t) -1 ) {
        cerr <<"can't set random seed 1\n";
        exit(1);
    }

    struct tm *tm = localtime(&timer);
    if (0 == tm ) {
        cerr << "can't set random seed 2\n";
        exit(1);
    }

    int seed = (1+tm->tm_sec) * (1+tm->tm_min) * (1+tm->tm_hour) * tm->tm_mday;
    srand(seed);
}

bool read_meta_data(map<string,int> &meta) {
  string path= "data/meta_data";
  ifstream meta_str;
  meta_str.open( path.c_str() );
  string line_buffer;
  if( meta_str.is_open() ) {
    string key;
    int val = 0;
    while( getline( meta_str, line_buffer) ) {
      istringstream line_stream(line_buffer);
      line_stream >> key >> val;
      meta.insert(std::make_pair(key,val));
    }
    return true;
  } else {
    return false;
  }
}
