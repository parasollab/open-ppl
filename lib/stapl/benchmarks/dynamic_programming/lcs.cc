/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/tuple.hpp>
#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/dynamic_programming.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/cross_view.hpp>
#include "test/test_report.hpp"
#include <fstream>
#include <string>
#include <iostream>

////////////////////////////////////////////////////////////////////
/// @file lcs.cc
///
/// Parallel implementation of the longest common subsequence (LCS) dynamic
/// programming problem using STAPL.
//////////////////////////////////////////////////////////////////////

#define LCS_MAX(a,b) (((a) > (b)) ? (a) : (b))

using namespace std;

using stapl::get;
using stapl::static_array;
using stapl::array_view;
using stapl::make_cross_view;
using stapl::default_timer;
using stapl::get_location_id;

//////////////////////////////////////////////////////////////////////
/// @brief Work function invoked on each element (i, j) in the 2D table which
///   represents the two input subsequences up to a point i for the first
///   sequence and point j for the second sequence.
///
/// The driver creates a PARAGRAPH using a @ref cross_view to represent the
/// table, and @ref sequence_comp_factory to populate the PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct dp_lcs_wf
{
  typedef T result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator invoked by the first task of the task graph and
  ///   processed the corner element of the table.
  //////////////////////////////////////////////////////////////////////
  template <typename ElementTuple>
  result_type
  operator()(ElementTuple e) const
  {
    return 0;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator invoked by the tasks processing the first row or
  ///   column elements of the table.
  /// @param e Table element to process.
  /// @param r1 Result of the task on the previous element in the row or column.
  //////////////////////////////////////////////////////////////////////
  template <typename ElementTuple, typename Res1>
  result_type operator()(ElementTuple e, Res1 r1) const
  {
    return 0;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator invoked by the tasks processing the interior
  ///   elements of the table.
  /// @param e Table element to process.
  /// @param r1,r2,r3, Result of the task on the previous element in the column,
  ///   row, and upper left diagonal, respectively, of the table.
  //////////////////////////////////////////////////////////////////////
  template <typename ElementTuple, typename Res1, typename Res2, typename Res3>
  result_type operator()(ElementTuple e, Res1 r1, Res2 r2, Res3 r3) const
  {
    typedef typename ElementTuple::value_type tuple_t;
    tuple_t const& tuple_ref = e;
    return LCS_MAX(r1,
                   LCS_MAX(r2,
                           r3 + (get<0>(tuple_ref) == get<1>(tuple_ref))));
  }
}; // struct dp_lcs_wf


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef static_array<char>    arrchr;

  typedef array_view<arrchr>    view_1D_arrchr;

  if (argc < 3 && get_location_id() == 0)
  {
    std::cerr << "Usage: mpirun -np <#procs> lcs <input_file> <block_size>\n";
    return EXIT_FAILURE;
  }

  const size_t block_size = atoi(argv[2]);

  if (get_location_id() == 0)
  {
    std::cout << "Block Size = " << block_size << "\n";
  }

  // Sequences are represented from elem_1 ... elem_(n-1)
  // as elem_0 is reserved as a special character
  //
  int cases;
  int accumDur = 0;

  ifstream myfile;

  myfile.open(argv[1]);

  myfile >> cases;

  string s1, s2;

  for (int c = 0; c < cases; ++c)
  {
    myfile >> s1 >> s2;

    arrchr a(s1.size()+1);
    arrchr b(s2.size()+1);

    if (stapl::get_location_id() == 0)
    {
      std::cout << "Sizes are " << s1.size() << ", " << s2.size() << "\n";

      a.set_element(0, '$');
      b.set_element(0, '$');

      for (size_t i=1; i <= s1.size(); ++i)
      {
        a.set_element(i, s1[i-1]);
      }

      for (size_t i=1; i <= s2.size(); ++i)
      {
        b.set_element(i, s2[i-1]);
      }
    }

    // stapl::rmi_fence();

    view_1D_arrchr va(a);
    view_1D_arrchr vb(b);

    auto cv2 = make_cross_view(va, vb);

  default_timer benchmark_timer;

  benchmark_timer.start();

  std::cout << stapl::get_location_id() << ": Starting Up\n";
  std::vector<std::vector<int> > result =
    stapl::paragraph<
      stapl::default_scheduler,
      stapl::sequence_comp_factory<dp_lcs_wf<int> >,
      decltype(cv2)
    >(stapl::sequence_comp_factory<dp_lcs_wf<int> >(dp_lcs_wf<int>(),
                                                    block_size),
      cv2)
    ();

  double benchmark_time = benchmark_timer.stop();

    if (stapl::get_location_id() == 0)
    {
      cout << "Length of LCS is " << result[2][0]
           << " computed in " << benchmark_time << endl;
    }
  }


  if (stapl::get_location_id() == 0)
  {
    std::cout << "Duration is: " << accumDur << endl;

    myfile.close();
  }

  return EXIT_SUCCESS;
}

