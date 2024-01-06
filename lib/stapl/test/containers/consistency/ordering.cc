/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/array_view.hpp>
#include <map>
#include <boost/unordered_set.hpp>
#include "../../test_report.hpp"

using namespace stapl;

template<typename T>
struct state
{
  typedef std::map<size_t, T> request_type;
  request_type requests;

  void update_value_from_location(size_t const& p, T const& v)
  {
#ifdef DEBUG
    std::cout << "loc: " << get_location_id() << " | from " << p
              <<  " | last value: " << requests[p] << " | new value: "
              << v << std::endl;
#endif
    // check if it's the next value i'm expecting
    if (requests[p] != v - 1 && v != 0)
      stapl_assert(false, "request out of order");

    // special case for 0
    if (v == 0 && requests[p] != 0)
      stapl_assert(false, "zeroth request out of order");

    // set the value
    requests[p] = v;
  }

  void define_type(typer& t)
  {
    t.member(requests);
  }
};

namespace stapl {

template<typename T, typename Accessor>
struct proxy<state<T>, Accessor>
  : private Accessor
{
  friend class proxy_core_access;
  typedef state<T> target_t;

  typedef typename target_t::request_type request_type;

public:
  explicit proxy(Accessor acc)
    : Accessor(acc)
  { }

  void update_value_from_location(size_t const& p, T const& v)
  {
    Accessor::invoke(&target_t::update_value_from_location, p, v);
  }
};

} // namespace stapl

template<typename T>
struct setter
{
  T m_value;
  size_t m_gid;

  typedef void result_type;

  setter(T const& t, size_t gid) : m_value(t), m_gid(gid) { }

  template<typename U>
  void operator()(U& u) const
  {
    u.update_value_from_location(m_value.first, m_value.second);
  }

  template<typename A>
  void operator()(const proxy<T, A>& u) const
  {
#ifdef DEBUG
    std::cout << "gid: " << m_gid << " | ";
#endif
    u.update_value_from_location(m_value.first, m_value.second);
  }

  void define_type(typer& t)
  {
    t.member(m_value);
    t.member(m_gid);
  }
};

template<typename G, typename T>
std::vector<std::pair<G, T> >
generate_requests(size_t const& n, size_t const& values, size_t const& round,
                  boost::unordered_set<size_t> const& locs,
                  boost::unordered_map<G, T>& running_requests)
{
  std::vector<std::pair<G, T> > requests;

  // if this location is participating this round
  if (locs.find(get_location_id()) != locs.end())
  {
    typedef indexed_domain<size_t>          domain_type;
    typedef balanced_partition<domain_type> partition_type;

    partition_type p(domain_type(0, n-1), get_num_locations());
    domain_type d = p[(get_location_id() + round) % get_num_locations()];

    // generate values requests for each gid
    for (size_t i = d.first(); i <= d.last(); ++i)
      for (size_t j = 0; j < values; j++)
        requests.push_back(std::make_pair(i, j));

    // shuffle the order of messages for a single gid
    std::random_shuffle(requests.begin(), requests.end());

    // make sure that the values for a single gid are in sorted order
    for (typename
           std::vector<std::pair<size_t, T> >::iterator it = requests.begin();
         it != requests.end(); ++it)
      (*it).second = running_requests[(*it).first]++;

#ifdef DEBUG
    for (typename
           std::vector<std::pair<G, T> >::iterator it = requests.begin();
         it != requests.end(); ++it)
      std::cout << get_location_id() << ": (" << (*it).first << " -> "
                << (*it).second << ")" << std::endl;
#endif
  }

#ifdef DEBUG
  if (requests.empty())
    std::cout << get_location_id() << ": sending nothing" << std::endl;
#endif

  return requests;
}

boost::unordered_set<size_t> participating_locations(size_t const& locs,
                                                     size_t const& round)
{
  boost::unordered_set<size_t> s;

  for (size_t i = 0; i < locs; ++i)
    s.insert((i + round) % get_num_locations());

#ifdef DEBUG
  std::cout << "round " << round << " locs: ";
  for (boost::unordered_set<size_t>::iterator it = s.begin();
       it != s.end(); ++it)
    std::cout << *it << " ";
  std::cout << std::endl;
#endif

  return s;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 5) {
    std::cerr << "usage: exe n participating_locations total_rounds num_values"
              << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);
  size_t l = atoi(argv[2]);
  const size_t r = atoi(argv[3]);
  const size_t v = atoi(argv[4]);

  if (l > get_num_locations())
    l = get_num_locations();

  typedef int        value_type;
  typedef state<int> state_type;
  array<state_type> c(n);

  boost::unordered_map<size_t, value_type> next_requests;

  // for r rounds
  for (size_t i = 0; i < r; ++i)
  {
    // generate a set of participating locations for the ith round
    boost::unordered_set<size_t> locs = participating_locations(l, i);

    // generate a local schedule of requests for the ith round
    std::vector<std::pair<size_t, value_type> > requests =
      generate_requests<size_t, value_type>(n, v, i, locs, next_requests);

    // go through the schedule and perform the operations one at a time
    for (std::vector<std::pair<size_t, value_type> >::iterator it =
           requests.begin();
         it != requests.end(); ++it)
      c.apply_set((*it).first,
                  setter<std::pair<size_t, value_type> >(
                    std::make_pair(get_location_id(), (*it).second),
                    (*it).first));
  }

  STAPL_TEST_REPORT(true, "Testing container ordering");

  return EXIT_SUCCESS;
}
