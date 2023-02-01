/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/profiler/base_profiler.hpp>
#include <stapl/unordered_map.hpp>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <string>
#include <vector>
#include <boost/random/mersenne_twister.hpp>

//for each
#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;
using namespace std;

template <typename T>
void printout(T s)
{
  stapl::do_once([&s]{
    std::cout << s;
  });
}

template <typename T>
void println(T s)
{
  stapl::do_once([&s]{
    std::cout << s << std::endl;
  });
}

//////////////////////////////////////////////////////////////////////
/// @brief This generates integer keys of a request key length.
//////////////////////////////////////////////////////////////////////
template <typename K>
struct keygen_wf
{
  typedef K key;

private:
  size_t m_keylength;

public:

  keygen_wf(size_t keylength)
    : m_keylength(keylength)
  {
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_keylength);
  }

  template<typename T>
  void operator()(T&& p)
  {
    int mod = 1;
    for (size_t i = 0; i<m_keylength; ++i)
    {
      mod *=10;
    }
    typedef std::uniform_int_distribution<int> rng_dist_t;
    std::random_device rd;
    std::mt19937 gen(rd());

    p = rng_dist_t(0, mod)(gen);
  }
};

template <typename K>
struct insert_wf
{
  typedef K key;

  template<typename Map, typename Key>
  void operator()(Map&& m, Key&&k)
  {
    m.insert(k, 81372);
  }
};

template <typename K>
struct erase_wf
{
  typedef K key;

  template<typename Map, typename Key>
  void operator()(Map&& m, Key&&k)
  {
    m.erase(k);
  }
};

template<typename T>
void eval(size_t keylength, size_t mapsize)
{
  typedef T key;
  typedef size_t data;

  typedef stapl::counter<stapl::default_timer> counter_t;

  counter_t timer;
  stapl::array<double> maptimes((size_t)get_num_locations());
  array_view<stapl::array<double>> maptimes_vw(maptimes);
  double maptime, total_time=0;
  size_t loc = stapl::get_location_id();

  stapl::vector<key> keys(mapsize);
  stapl::vector_view<stapl::vector<key>> key_vw(keys);

  map_func(keygen_wf<T>(keylength), key_vw);
  printout("  Keys Generated: ");
  println(std::to_string(keys.size()));

  // Profile Unordered_Map
  stapl::unordered_map<key, data> m;
  stapl::map_view<stapl::unordered_map<key, data>> map_vw(m);

  // Insert
  timer.reset();
  timer.start();
  map_func(insert_wf<T>(), make_repeat_view(map_vw), key_vw);
  maptimes[loc] = timer.stop();
  maptime = stapl::reduce(maptimes_vw, stapl::max<double>());
  total_time += maptime;

  printout("  Insert() Time (map_func): ");
  println(std::to_string(maptime));
  printout("    map Size: ");
  println(std::to_string(m.size()));

  // Clear
  timer.reset();
  timer.start();
  m.clear();
  maptimes[loc] = timer.stop();
  maptime = stapl::reduce(maptimes_vw, stapl::max<double>());
  total_time += maptime;

  printout("  Clear() Time: ");
  println(std::to_string(maptime));

  // Reinsert so we can do find and erase.
  map_func(insert_wf<T>(), make_repeat_view(map_vw), key_vw);

  // Find
  timer.reset();
  T findkey = keys[mapsize/2];
  timer.start();
  m.find(findkey);
  maptimes[loc] = timer.stop();
  maptime = stapl::reduce(maptimes_vw, stapl::max<double>());
  total_time += maptime;
  printout("  Find(Key) Size: ");
  println(std::to_string(maptime));

  // Size
  timer.reset();
  timer.start();
  m.size();
  maptimes[loc] = timer.stop();
  maptime = stapl::reduce(maptimes_vw, stapl::max<double>());
  total_time += maptime;
  printout("  Size() Time: ");
  println(std::to_string(maptime));

  // Erase
  timer.reset();
  timer.start();
  map_func(erase_wf<T>(), make_repeat_view(map_vw), key_vw);
  maptimes[loc] = timer.stop();
  maptime = stapl::reduce(maptimes_vw, stapl::max<double>());
  total_time += maptime;

  printout("  Erase(Key) Time (map_func): ");
  println(std::to_string(maptime));
  printout("    map Size: ");
  println(std::to_string(m.size()));

  // Total Time
  printout("Total Time: ");
  println(std::to_string(total_time));

}

stapl::exit_code stapl_main(int argc,char** argv)
{
  int myid  =stapl::get_location_id();
  size_t case_id, keylength, mapsize;
  if (argc < 4) {
    case_id = 0;
    keylength = 10;
    mapsize = 2;
  }
  else {
    case_id = atol(argv[1]);
    keylength = atol(argv[2]);
    mapsize  = atol(argv[3]);
  }

  if (myid == 0) {
    cout << "Case ID=" << case_id << "\n";
    cout << " Key Length:" << keylength << "\n";
    cout << " map Size:" << mapsize << "\n";
  }

  srand(myid);

  stapl::do_once([]{
    printf("unordered_map<int,int>\n");
  });
  eval<int>(keylength, mapsize);

  return EXIT_SUCCESS;

}
