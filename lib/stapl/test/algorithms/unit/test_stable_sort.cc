/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <boost/lexical_cast.hpp>

#include <stdlib.h>

enum {LESS, MORE};

typedef long long ll;
typedef std::pair<ll, ll> ll_pair;
typedef stapl::array<ll_pair> ll_array;
typedef stapl::array_view<ll_array> ll_array_view;
typedef ll_array_view::iterator ll_arrv_iterator;

//////////////////////////////////////////////////////////////////////
/// @brief comparator that handles both '<' and '>' operators
//         and checks that the second element of the pair is
//         in increasing order
/// @tparam o enum value indicating the operator to use
//////////////////////////////////////////////////////////////////////
struct first_element_comparator
{
  using first_argument_type  = ll_pair;
  using second_argument_type = ll_pair;

  int op;
  ll last_key;
  ll last_value;

  first_element_comparator(int o)
    : op(o), last_key(-1), last_value(-1)
  {}

  template <typename LRef, typename RRef>
  bool operator () (LRef const& a, RRef const& b) const
  {
    return (op == LESS)
           ? a.first < b.first
           : a.first > b.first;
  }

  template <typename R>
  bool check(R const& new_value)
  {
    if (new_value.first != last_key)
    {
      last_key = new_value.first;
      last_value = new_value.second;
      return true;
    }

    if (new_value.second > last_value)
    {
      last_value = new_value.second;
      return true;
    } else {
      return false;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(op);
    t.member(last_key);
    t.member(last_value);
  }

};

stapl::exit_code stapl_main (int argc, char **argv) {

  srand(time(NULL));

  ll num = boost::lexical_cast<ll> (argc > 1 ? argv[1] : "16");
  ll mod = boost::lexical_cast<ll> (argc > 2 ? argv[2] : "4");

  stapl::counter<stapl::default_timer> timer;
  timer.reset();

  ll_array arr_less(num);
  ll_array_view array_less_view(arr_less);

  ll_array arr_more(num);
  ll_array_view array_more_view(arr_more);

  for (ll i = 0; i < num; i++)
  {
    array_less_view.set_element(i, std::make_pair(i % mod, i));
    array_more_view.set_element(i, std::make_pair(i % mod, i));
  }

  first_element_comparator less_comparator(LESS);
  first_element_comparator more_comparator(MORE);

  timer.start();
  stapl::stable_sort(array_less_view, less_comparator);
  timer.stop();
  double time_less = timer.value();

  timer.reset();
  timer.start();
  stapl::stable_sort(array_more_view, more_comparator);
  timer.stop();
  double time_more = timer.value();

  bool error = false;

  stapl::do_once ( [&] {

    ll_arrv_iterator it_less = array_less_view.begin();
    ll_arrv_iterator it_more = array_more_view.begin();

    for(ll i = 0; i < num && !error; i++)
    {

       ll_pair less_value = *(it_less++);
       ll_pair more_value = *(it_more++);

       if(! less_comparator.check(less_value))
       {
         std::cerr << "Error: Unstable sort (LESS). current: ("
                   << less_value.first << ":"
                   << less_value.second << ")"
                   << std::endl;
         error = true;
       }

       if(! more_comparator.check(more_value))
       {
          std::cerr << "Error: Unstable sort (MORE). current: ("
                    << more_value.first << ":"
                    << more_value.second << ")"
                    << std::endl;
          error = true;
        }

    }

    std::cerr << "Tests " << (error ? "failed" : "passed")
              << ". sort time (less):" << time_less
              << "; sort time (more):" << time_more
              << std::endl;

  });

  if(error)
  {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;

}
