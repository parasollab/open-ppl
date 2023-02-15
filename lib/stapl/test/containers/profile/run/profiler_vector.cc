/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/vector.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include "../workfunctions.hpp"
#include "../profile.hpp"
#include "../seq_container.hpp"
#include "../read_write.hpp"
#include "../subscript.hpp"
#include "../sequence.hpp"
#include "../locality.hpp"
#include "../restructure.hpp"
#include "../vector.hpp"

#include <boost/lexical_cast.hpp>

#include "../profiling_util.hpp"
#include "../value_type_util.hpp"

using namespace stapl;
using namespace profiling;

////////////////////////////////////////////////////////////////////////////////
/// @brief Populate the initial values.
////////////////////////////////////////////////////////////////////////////////
template<typename Vals,
  bool = is_container<Vals>::value,
  bool = std::is_convertible<typename Vals::value_type, int>::value>
struct value_initializer
{
  static void apply(Vals& v)
  {
    auto vw = make_array_view(v);
    stapl::iota(vw, 0);
  }
};

template<typename Vals>
struct value_initializer<Vals, true, false>
{
  static void apply(Vals& v)
  {
    auto vw = make_array_view(v);
    stapl::fill(vw, typename Vals::value_type());
  }
};

template<typename Vals>
struct value_initializer<Vals, false, true>
{
  static void apply(Vals& v)
  {
    std::iota(v.begin(), v.end(), 0);
  }
};

template<typename Vals>
struct value_initializer<Vals, false, false>
{
  static void apply(Vals& v)
  {
    std::fill(v.begin(), v.end(), typename Vals::value_type());
  }
};

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the methods of given container.
///
/// @tparam Cont Type of the container
/// @tparam DS   Type determining the distribution of the container
///
/// @param name A string identifier of the container
/// @param ds Distribution specification (size in the simplest case, in which
///           the default distribution will be used, but can also be
///           a partitioner instance or a view-based distribution specification)
/// @param first   Index of the first element on current location
/// @param sz      Number of elements on current location
/// @param indices Indices of the elements to be processed during the profile
///                runs
/// @param nremote Number of indices that are remote for current location (for
///                testing locality queries)
/// @param vals    Values to be written at the positions specified by @a indices
/// @param argc, argv Command line args. with additional profiling parameters
////////////////////////////////////////////////////////////////////////////////
template<typename Cont, typename DS>
void profile_container(std::string const& name, DS const& ds,
                       size_t first, size_t sz,
                       std::vector<size_t> const& indices, size_t nremote,
                       std::vector<typename Cont::value_type> const& vals,
                       int argc, char** argv)
{
  using value_type = typename Cont::value_type;

  Cont cont(ds);
  value_initializer<Cont>::apply(cont);

  get_op<value_type> get_fn;
  set_op<value_type> set_fn(vals[0]);

  auto p = set_common_container_profilers(name, cont, ds, argc, argv);
  p.push_back(new constructor_size_value_profiler<Cont, DS>(name, ds));

  add_locality_profilers(p, name, cont, indices, nremote, argc, argv);
  add_sequence_profilers(p, name, cont, first, sz, nremote, vals, argc, argv);
  add_read_profilers(p, name, cont, get_fn, indices, argc, argv);
  add_write_profilers(p, name, cont, set_fn, indices, vals, argc, argv);

  add_subscript_read_profilers(p, name,
    cont, indices, validators::subscript<Cont>(), argc, argv);
  add_subscript_write_profilers(p, name,
    cont, indices, vals, validators::subscript<Cont>(vals), argc, argv);

  add_vector_profilers(p, name, cont, indices, vals, argc, argv);
  add_resize_profilers(p, name, cont, argc, argv);

  profile_and_clear(p);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the methods of the most common views that
///        can be defined over a given container.
///
/// @tparam Cont Type of the underlying container
/// @tparam DS   Type determining the distribution of the underlying container
///
/// @param cont_name  A string with the name of the underlying container
/// @param ds         Distribution specification for the underlying container
///                   (@sa profile_container)
/// @param ind_1   Primary set of indices to be processed
/// @param ind_2   Secondary set of indices to be processed (used for profiling
///                the @p advance() or @p distance() methods, for example)
/// @param nremote Number of indices in the primary set that are remote for
///                current location (for testing locality queries)
/// @param vals    Values to be written at the positions specified by @a ind_1
/// @param argc, argv Command line args. with additional profiling parameters
////////////////////////////////////////////////////////////////////////////////
template<typename Cont, typename DS>
void profile_views(std::string const& cont_name, DS const& ds,
                   size_t first, size_t sz,
                   std::vector<size_t> const& ind_1,
                   std::vector<size_t> const& ind_2,
                   size_t nremote,
                   std::vector<typename Cont::value_type> const& vals,
                   int argc, char** argv)
{
  using value_type = typename Cont::value_type;

  Cont cont(ds);
  value_initializer<Cont>::apply(cont);

  std::string name = "vector_view<" + cont_name + ">";
  auto vw1 = make_vector_view(cont);
  auto p1 = set_common_view_profilers(name, vw1, ds, argc, argv);

  get_op<value_type> get_fn;
  set_op<value_type> set_fn(vals.front());

  using subscript_validator1 = validators::subscript<decltype(vw1)>;

  add_locality_profilers(p1, name, vw1, ind_1, nremote, argc, argv);
  add_sequence_profilers(p1, name,
    vw1, first, sz, nremote, ind_1, ind_2, vals, argc, argv);
  add_read_profilers(p1, name, vw1, get_fn, ind_1, argc, argv);
  add_write_profilers(p1, name, vw1, set_fn, ind_1, vals, argc, argv);
  add_subscript_read_profilers(p1, name,
    vw1, ind_1, subscript_validator1(), argc, argv);
  add_subscript_write_profilers(p1, name,
    vw1, ind_1, vals, subscript_validator1(vals), argc, argv);
  add_vector_profilers(p1, name, vw1, ind_1, vals, argc, argv);
  add_resize_profilers(p1, name, vw1, argc, argv);

  profile_and_clear(p1);

  name = "array_view<" + name + ">";
  auto vw2 = make_array_view(vw1);
  auto p2 = set_common_view_profilers(name, vw2, ds, argc, argv);

  using subscript_validator2 = validators::subscript<decltype(vw2)>;

  add_locality_profilers(p2, name, vw2, ind_1, nremote, argc, argv);
  add_sequence_profilers(p2, name,
    vw2, first, sz, nremote, ind_1, ind_2, vals, argc, argv);
  add_read_profilers(p2, name, vw2, get_fn, ind_1, argc, argv);
  add_write_profilers(p2, name, vw2, set_fn, ind_1, vals, argc, argv);
  add_subscript_read_profilers(p2, name,
    vw2, ind_1, subscript_validator2(), argc, argv);
  add_subscript_write_profilers(p2, name,
    vw2, ind_1, vals, subscript_validator2(vals), argc, argv);

  profile_and_clear(p2);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Calls the profilers for the methods of given container and the most
///        common views that can be defined over it.
///
/// @tparam Cont Type of the container
/// @tparam DS   Type determining the distribution of the container
///
/// @param cont_name  A string with the name of the container
/// @param ds         Distribution specification for the container
///                   (@sa profile_container)
/// @param ind_1   Primary set of indices to be processed
/// @param ind_2   Secondary set of indices to be processed (used for profiling
///                the @p advance() or @p distance() methods, for example)
/// @param nremote Number of indices in the primary set that are remote for
///                current location (for testing locality queries)
/// @param argc, argv  Command line args. with additional profiling parameters
////////////////////////////////////////////////////////////////////////////////
template<typename Cont, typename DS>
void profile_all(std::string const& cont_name, DS const& ds,
                 size_t first, size_t sz,
                 std::vector<size_t> const& ind_1,
                 std::vector<size_t> const& ind_2,
                 size_t nremote,
                 int argc, char** argv)
{
  std::vector<typename Cont::value_type> vals(sz);
  value_initializer<decltype(vals)>::apply(vals);

  profile_container<Cont>(
    cont_name, ds, first, sz, ind_1, nremote, vals, argc, argv);
  profile_views<Cont>(
    cont_name, ds, first, sz, ind_1, ind_2, nremote, vals, argc, argv);
}

stapl::exit_code stapl_main(int argc, char** argv)
{
  stapl_print("\n"
    "Unit tests/profiling of the methods of vector and associated views."
    "\n\n");

  if (argc == 2 && !strcmp("--help", argv[1]))
  {
    print_base_profiler_cmdline_help(std::cout);
    print_adt_profilers_cmdline_help(std::cout);
    print_rw_profilers_cmdline_help(std::cout);
    print_index_generator_cmdline_help(std::cout);
    return EXIT_SUCCESS;
  }

  int case_id = 0;
  size_t nelems = 10;

  if (argc > 2)
    nelems = boost::lexical_cast<std::size_t>(argv[2]);
  else
    stapl_print("Input size not specified. Using 10.\n");

  if (argc > 1)
    case_id = atoi(argv[1]);
  else
    stapl_print(
      "Test case not specified. Using default-distributed vector<int>\n");

  // Generate indices to be processed during the testing. A range of indices
  // is specified at each location and two sets of indices with uniform random
  // distribution in that range are generated. Percentage of remote elements
  // and whether the remote elements are only allowed to be on the adjacent
  // locations can be prescribed via command line arguments.

  size_t block = nelems / get_num_locations();
  size_t first = block*get_location_id();

  size_t premote;
  bool next_only;
  std::tie(premote, next_only) = determine_index_generator_param(argc, argv);

  rand_gen rnd(nelems);

  auto indices1 = generate_indices<size_t>(
    rnd, first, block, premote, next_only);

  size_t nremote1 = premote * block / 100;  // number of remote elements

  auto indices2 = generate_indices<size_t>(
    rnd, first, block, premote, next_only);

  // Test the sequential containers

  if (get_num_locations() == 1)
  {
    if (case_id % 2 == 0)
      profile_seq_container<std::vector<int>>("std::vector<int>",
        nelems, indices1, argc, argv);
    else
      profile_seq_container<std::vector<MVT>>("std::vector<MVT>",
        nelems, indices1, argc, argv);
  }

  // Test STAPL containers/views

  if (case_id==0 || case_id==1) // default distribution
  {
    if (case_id==0)
    {
      profile_all<vector<int>>("vector<int>",
        nelems, first, block, indices1, indices2, nremote1, argc, argv);
    }
    else // case_id=1
    {
      profile_all<vector<MVT>>("vector<MVT>",
        nelems, first, block, indices1, indices2, nremote1, argc, argv);
    }
  }
  else if (case_id==2 || case_id==3)  // block distribution
  {
    size_t constexpr NBS=4;
    size_t bs[NBS] = {1024, 8192, 65536, 0};
    bs[NBS-1] = nelems/get_num_locations();

    for (size_t i=0; i<NBS; ++i)
    {
      if (bs[i] > nelems)
      {
        if (stapl::get_location_id() == 0)
          std::cout << "Block size " << bs[i] << " > size of the container ("
                    << nelems << ")" << std::endl;
      }

      std::ostringstream o;
      o << bs[i];

      using domain_type = indexed_domain<size_t>;
      using partition_type = block_partitioner<domain_type>;

      partition_type blocked(domain_type(0, nelems-1, true), bs[i]);

      if (case_id==2)
      {
        profile_all<vector<int, partition_type>>(
          "vector<int, blocked("+ o.str() +")>",
          blocked, first, block, indices1, indices2, nremote1, argc, argv);
      }
      else // case_id==3
      {
        profile_all<vector<MVT, partition_type>>(
          "vector<MVT, blocked("+o.str()+")>",
          blocked, first, block, indices1, indices2, nremote1, argc, argv);
      }

      rmi_fence();
    }
  }

  return EXIT_SUCCESS;
}
