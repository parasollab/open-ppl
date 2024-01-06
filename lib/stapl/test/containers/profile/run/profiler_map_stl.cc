/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/profiler/base_profiler.hpp>
#include <stapl/map.hpp>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <string>
#include <vector>
#include <boost/random/mersenne_twister.hpp>

//for each
#include <stapl/algorithms/algorithm.hpp>

#include <iostream>
#include <map>
#include <vector>
#include <array>
#include <string>
#include <vector>
#include <boost/random/mersenne_twister.hpp>

using namespace stapl;
using namespace std;

template <typename T>
void printout(T s)
{
  std::cout << s;
}
template <typename T>
void println(T s)
{
  std::cout << s << std::endl;
}

template<typename T>
void eval(size_t keylength, size_t mapsize)
{
  typedef T key;
  typedef size_t data;

  typedef stapl::counter<stapl::default_timer> counter_t;

  counter_t timer;
  double maptime, total_time=0;;

  std::vector<key> keys(0);

  // Generate keys
  for (size_t i=0; i< mapsize; ++i)
  {
    int mod= 1;
    for (size_t i=0; i<keylength; ++i)
    {
      mod *=10;
    }
    typedef std::uniform_int_distribution<int> rng_dist_t;
    std::random_device rd;
    std::mt19937 gen(rd());

    int p = rng_dist_t(0, mod)(gen);
    keys.push_back(p);
  }
  printout("  Keys Generated: ");
  println(std::to_string(keys.size()));

  // Profile Map
  std::map<key, data> m;

  // Insert
  timer.reset();
  timer.start();
  for (size_t i=0; i< mapsize; ++i)
  {
    m[keys[i]] = 81372;
  }
  maptime = timer.stop();
  total_time += maptime;

  printout("  Insert() Time (map_func): ");
  println(std::to_string(maptime));
  printout("    map Size: ");
  println(std::to_string(m.size()));

  // Clear
  timer.reset();
  timer.start();
  m.clear();
  maptime = timer.stop();
  total_time += maptime;


  printout("  Clear() Time: ");
  println(std::to_string(maptime));

  // Find
  timer.reset();
  int findkey = keys[mapsize/2];
  timer.start();
  m.find(findkey);
  maptime = timer.stop();
  total_time += maptime;
  printout("  Find(Key) Size: ");
  println(std::to_string(maptime));

  // Size
  timer.reset();
  timer.start();
  m.size();
  maptime = timer.stop();
  total_time += maptime;
  printout("  Size() Time: ");
  println(std::to_string(maptime));

  // Erase
  timer.reset();
  timer.start();
  for (size_t i=0; i< mapsize; ++i)
  {
    m.erase(keys[i]);
  }

  maptime = timer.stop();
  total_time += maptime;

  printout("  Erase(Key) Time: ");
  println(std::to_string(maptime));
  printout("    map Size: ");
  println(std::to_string(m.size()));

  printout("Total Time: ");
  println(std::to_string(total_time));

}

int main(int argc,char** argv)
{
  size_t case_id, keylength, mapsize;
  if (argc < 4) {
    case_id = 0;
    keylength = 10;
    mapsize = 100;
  }
  else {
    case_id = atol(argv[1]);
    keylength = atol(argv[2]);
    mapsize  = atol(argv[3]);
  }

  cout << "Case ID=" << case_id << "\n";
  cout << " Key Length:" << keylength << "\n";
  cout << " map Size:" << mapsize << "\n";


  if (case_id == 0) {
    printout("map<int,int>\n");
    eval<int>(keylength, mapsize);
  }
  else {
    println("Invalid case\n");
  }

  return 0;
}
