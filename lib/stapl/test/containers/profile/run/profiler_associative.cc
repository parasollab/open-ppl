/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include  <boost/type_traits/remove_const.hpp>
#include "../p_container_profiler.hpp"
#include "../p_container_algo_profiler.hpp"
#include "../profile_algo.h"
#include "stapl/containers/map/map.hpp"
#include "stapl/containers/set/set.hpp"
#include "stapl/containers/partitions/blocked_partition.hpp"

using namespace stapl;

typedef size_t MYKEY;
typedef long int DATA;

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns the value from a variable.
///
/// @tparam Val The value type
////////////////////////////////////////////////////////////////////////////////
template <class Val>
struct AGenerator
{
  typedef std::vector<typename boost::remove_const<Val>::type> vtype;
  static Val extract(size_t idx)
  {
    return idx;
  } //for simple
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of values: an index and a random number.
///
/// @tparam Ret The index type
/// @tparam Val The random number type
////////////////////////////////////////////////////////////////////////////////
template <class Ret, class Val>
struct AGenerator<std::pair<Ret,Val> >
{
  typedef std::pair<typename boost::remove_const<Ret>::type,Val> value_type;
  typedef std::vector<value_type> vtype;
  static value_type extract(size_t idx)
  {
    return value_type(idx, rand() );
  } //for pair
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Populates a container with index values.
////////////////////////////////////////////////////////////////////////////////
class index_generator
{
public:

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Populates a container with index values.
  ///
  /// @param indices The container that holds the resulting indices
  /// @param first The starting value for the current location
  /// @param sz The size of the container
  /// @param per_remote The percentage of indices that are remote
  /// @param next_only Restricts the remote indices to neighbors if 1.
  //////////////////////////////////////////////////////////////////////////////
  template <class V>
  static void generate(V& indices, size_t first, size_t sz,
                       size_t per_remote, size_t next_only)
  {
    //if non local are on neighbours only (1) else on all Ps(0)
    size_t nLocs = get_num_locations();
    typedef AGenerator<typename V::value_type> AG;
    indices.resize(sz);

    //generate first as if all are local
    for (size_t i = 0; i < sz; ++i) {
      indices[i] = AG::extract(first + ( lrand48() % sz));
    }

    if (per_remote == 0) {//all indices local
      return;
    }
    else { //there is a remote percentage
      size_t nremote = (per_remote * sz) / 100;
      size_t bs = sz / nremote;
      if (next_only == 1) { //remote only on neighbor
        for (size_t i = 0; i < nremote; i+=bs) {
          indices[i] =
            AG::extract( (sz + first + ( lrand48() % sz)) % (nLocs * sz));
        }
      }
      else { //al over the place
        for (size_t i = 0; i < nremote; i+=bs) {
          indices[i] = AG::extract( lrand48() % (nLocs * sz) );
        }
      }
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the functions of the associative containers.
///
/// @param pcname A string that contains the name of the container
/// @param p The container
/// @param local_data The index values
////////////////////////////////////////////////////////////////////////////////
template <class pC, class sC>
void profile(std::string pcname, pC& p, sC& local_data, int argc,char** argv)
{
  assoc_find_val_profiler<pC, sC, counter_type>
    afvp(pcname, &p, &local_data, argc, argv);
  afvp.collect_profile();
  afvp.report();

  assoc_insert_profiler<pC, sC, counter_type>
    aip(pcname, &p, &local_data, argc, argv);
  aip.collect_profile();
  aip.report();

  assoc_find_profiler<pC, sC, counter_type>
    afp(pcname, &p, &local_data, argc, argv);
  afp.collect_profile();
  afp.report();

// can be uncommented once erase is implemented in the distribution
//  assoc_erase_profiler<pC, sC, counter_type>
//    aep(pcname, &p, &local_data, argc, argv);
//  aep.collect_profile();
//  aep.report();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Initialize the index values and the container and invoke the profiler
/// tests.
///
/// @param pcname A string containing the name of the conatiner
/// @param NElems The size of the container
////////////////////////////////////////////////////////////////////////////////
template <class pC>
void evaluate(std::string pcname, size_t NElems, int argc, char** argv)
{
  typedef typename pC::value_type value_type;

  //generate test data; it will be inserted, searched,
  size_t nLocs = get_num_locations();
  typedef AGenerator<value_type> AG;
  typedef typename AG::vtype sv_type;
  sv_type local_data;
  size_t block = NElems / nLocs;

  size_t premote = 0;
  size_t next_only = 1;
  for ( int i = 1; i < argc; i++)
  {
    if ( !strcmp("--premote", argv[i]) ) {
      premote = atoi(argv[++i]);
    }
    if ( !strcmp("--next_only", argv[i]) ) {
      next_only = atoi(argv[++i]);
    }
  }

  index_generator::generate(local_data, block*get_location_id(),block,
                            premote,next_only);

  //allocate the empty container
  typedef indexed_domain<size_t> domain_type;
  domain_type d(0, NElems-1);
  pC p(d);
  profile(pcname, p, local_data, argc, argv);
}

stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t NElems;
  if (get_location_id()==0)
    stapl_print("Associative pContainer Performance Evaluation\n");
  if (argc > 1) {
    NElems = atoi(argv[1]);
    srand(NElems + get_location_id());
    srand48(NElems + get_location_id());
  }
  else {
    stapl_print("Case and Input size required; Using 0 and 10 by default\n");
    NElems=10;
   // case_id = 0; // Uncomment and create variable when more cases will exist.
  }

  if (get_location_id()==0)
    std::cout << "Evaluating assoc pcontainer methods using P=" <<
      get_num_locations() << " N=" << NElems << "\n";

  stapl_print("===========Evaluating map\n");
  evaluate<map<MYKEY, DATA> >("map", NElems, argc, argv);

  stapl_print("===========Evaluating set\n");
  evaluate<set<MYKEY> >("set", NElems, argc, argv);

  rmi_fence();

  return EXIT_SUCCESS;

}
