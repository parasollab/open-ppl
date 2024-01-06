/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <cstdlib>
#include <cmath>
#include <string>
#include <iostream>
#include <limits>

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/functional.hpp>

#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include "json_parser.hpp"

#include "testutil.hpp"
#include "rel_alpha_data.h"

using namespace std;

//////////////////////////////////////////////////////////////////////

typedef stapl::negate<int> ineg_wf;
typedef stapl::max<int> imax_wf;
typedef stapl::identity<int> iid_wf;
typedef stapl::plus<int> iadd_wf;
typedef stapl::min<int> imin_wf;
typedef stapl::logical_or<int> ior_wf;
typedef stapl::minus<int> isub_wf;
typedef stapl::multiplies<int> imul_wf;
typedef stapl::equal_to<int> ieq_wf;

//////////////////////////////////////////////////////////////////////

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_view_tp;

typedef ary_int_view_tp::domain_type dom_tp;

//////////////////////////////////////////////////////////////////////

template <typename T>
struct odd_value
{
  typedef T    argument_type;
  typedef bool result_type;

  bool operator() (const T& x) const
  {
    return 1 == (x % 2);
  }
};

template <typename T>
struct even_value
{
  typedef T    argument_type;
  typedef bool result_type;

  bool operator() (const T& x) const
  {
    return 0 == (x % 2);
  }
};

//////////////////////////////////////////////////////////////////////

void init_int( ary_int_tp &, ary_int_tp &,
               ary_int_tp &, ary_int_tp &,
               ary_int_tp &, ary_int_tp &,
               ary_int_tp &, ary_int_tp &,
               ary_int_tp &, ary_int_tp &, int );

bool test_001( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_002( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_003( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_004( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_005( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_006( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_007( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_008( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_009( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_010( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_011( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_012( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_013( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_014( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_015( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_016( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_017( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_018( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_019( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_020( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_021( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_022( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_023( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_024( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_025( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_026( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_027( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_028( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_029( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_030( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_031( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_032( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_033( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_034( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_035( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_036( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_037( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_038( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_039( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_040( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_041( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_042( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_043( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_044( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_045( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_046( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_047( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_048( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_049( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );
bool test_050( ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &, ary_int_tp &, ary_int_tp &,
                  ary_int_tp &, ary_int_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &,
                  ary_int_view_tp &, ary_int_view_tp &, int );

//////////////////////////////////////////////////////////////////////
// DATA PARALLEL TESTS - 1 DIMENSIONAL ARRAY
//////////////////////////////////////////////////////////////////////

bool opt_list = false, opt_noref = false, opt_out = false, opt_quiet = false;
int opt_test = -1;
char *opt_data = 0;
bool opt_stapl = false;
bool opt_manual = false;

stapl::exit_code stapl_main(int argc, char **argv) {

  char *temp = 0;
  for (int argi = 1; argi < argc; ) {
    char * opt = argv[argi++];
    if ('-' == opt[0] ) {
      switch( opt[1] ) {
      case 'h':
        cerr << "HELP\n";
        break;
      case 'd':
        opt_data = argv[argi++];
        break;
      case 'l':
        opt_list = true;
        break;
      case 'n':
        opt_noref = true;
        break;
      case 'o':
        opt_out = true;
        break;
      case 'q':
        opt_quiet = true;
        break;
      case 't':
        temp = argv[argi++];
        opt_test = atoi(temp);
        break;
      case 'v':
        temp = argv[argi++];
        opt_stapl = (0 == strcmp("stapl", temp) );
        opt_manual = (0 == strcmp("manual", temp) );
        break;
      }
    } else {
      cerr << "unknown command line argument " << opt << endl;
    }
  }
  if (!opt_stapl && !opt_manual ) {
    // default behavior
    opt_stapl = true;
    opt_manual = true;
  }

  if (opt_list ) {
    // Boost.PropertyTree will only serialize a node with unnamed subkeys
    // as an array.  Therefore we've got to build extra trees in order to
    // create the unnamed subkeys needed to form proper JSON output.
    typedef boost::property_tree::ptree ptree;
    ptree manual_tree;
    ptree stapl_tree;
    manual_tree.push_back(std::make_pair("name", ptree("stapl")));
    stapl_tree.push_back(std::make_pair("name", ptree("manual")));
    ptree version_array;
    version_array.push_back(std::make_pair("", manual_tree));
    version_array.push_back(std::make_pair("", stapl_tree));

    ptree test_array;
    for (int i = 1; i<50; i++ ) {
      ptree test;
      char buf[8];
      sprintf(buf,"t%03d",i);
      test.push_back(std::make_pair(std::string(buf), version_array));
      test_array.push_back(std::make_pair("", test));
    }

    // The array of tests is the property of a key named "tests".
    ptree test_tree;
    test_tree.push_back(std::make_pair("tests", test_array));
    write_json(std::cout, test_tree, false);
    return EXIT_SUCCESS;
  }

  int model = -1;
  switch( opt_data[0] ) {
  case 't':
    model = 1;
    break;
  case 's':
    model = 100;
    break;
  case 'm':
    model = 10000;
    break;
  case 'b':
    model = 10000000;
    break;
  case 'h':
    model = 100000000;
    break;
  default:
    cerr << "opt_data " << opt_data << endl;
    break;
  }
  if (model == -1) {
    std::cerr << "usage: exe -data tiny/small/medium/big/huge\n";
    exit(1);
  }

  int first_test = 1;
  int last_test = 50;
  if (opt_test != -1 ) {
    first_test = opt_test;
    last_test = opt_test;
  }

  int size = 1000 * model;
  ary_int_tp a(size), b(size), c(size), d(size);
  ary_int_tp e(size), f(size), g(size), h(size);
  ary_int_tp p(size), q(size);

  ary_int_view_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);
  ary_int_view_tp e_vw(e), f_vw(f), g_vw(g), h_vw(h);
  ary_int_view_tp p_vw(p), q_vw(q);

  init_int( a, b, c, d, e, f, g, h, p, q, model);

  int fail_count = 0;
  bool ok = true;
  for (int test = first_test; test <= last_test; test++ ) {
    switch (test) {
    case 1:
      ok = test_001(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 2:
      ok = test_002(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 3:
      ok = test_003(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 4:
      ok = test_004(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 5:
      ok = test_005(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 6:
      ok = test_006(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 7:
      ok = test_007(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 8:
      ok = test_008(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 9:
      ok = test_009(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 10:
      ok = test_010(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 11:
      ok = test_011(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 12:
      ok = test_012(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 13:
      ok = test_013(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 14:
      ok = test_014(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 15:
      ok = test_015(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 16:
      ok = test_016(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 17:
      ok = test_017(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 18:
      ok = test_018(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 19:
      ok = test_019(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 20:
      ok = test_020(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 21:
      ok = test_021(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 22:
      ok = test_022(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 23:
      ok = test_023(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 24:
      ok = test_024(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 25:
      ok = test_025(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 26:
      ok = test_026(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 27:
      ok = test_027(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 28:
      ok = test_028(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 29:
      ok = test_029(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 30:
      ok = test_030(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 31:
      ok = test_031(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 32:
      ok = test_032(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 33:
      ok = test_033(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 34:
      ok = test_034(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 35:
      ok = test_035(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 36:
      ok = test_036(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 37:
      ok = test_037(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 38:
      ok = test_038(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 39:
      ok = test_039(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 40:
      ok = test_040(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 41:
      ok = test_041(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 42:
      ok = test_042(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 43:
      ok = test_043(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 44:
      ok = test_044(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 45:
      ok = test_045(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 46:
      ok = test_046(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 47:
      ok = test_047(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 48:
      ok = test_048(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 49:
      ok = test_049(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    case 50:
      ok = test_050(a, b, c, d, e, f, g, h, p, q,
              a_vw,b_vw,c_vw,d_vw,e_vw,f_vw,g_vw,h_vw,p_vw,q_vw, size);
      break;
    }
  }

  return EXIT_SUCCESS;
}

//////////////////////////////////////////////////////////////////////
// initialize integer arrays
//////////////////////////////////////////////////////////////////////

void init_int( ary_int_tp & a, ary_int_tp & b,
               ary_int_tp & c, ary_int_tp & d,
               ary_int_tp & e, ary_int_tp & f,
               ary_int_tp & g, ary_int_tp & h,
               ary_int_tp & p, ary_int_tp & q, int model) {

  for (int i = 0; i < model; i++) {
    for (int j = 0; j < 1000; j++) {
      a[i*1000+j] = prime1000[rand1000_01[j]]; // integer in
      b[i*1000+j] = prime1000[rand1000_02[j]]; // integer in
      c[i*1000+j] = prime1000[rand1000_03[j]] % 2; // bool in
      d[i*1000+j] = prime1000[rand1000_04[j]] % 2; // bool in
      e[i*1000+j] = prime1000[rand1000_05[j] % 100 ]; // dup integer in
      f[i*1000+j] = prime1000[rand1000_06[j] % 100 ]; // dup integer in
      g[i*1000+j] = 0; // temp/output
      h[i*1000+j] = 0; // temp/output
      p[i*1000+j] = 0; // temp/output
      q[i*1000+j] = 0; // temp/output
    }
  }
}

//////////////////////////////////////////////////////////////////////
// FEATURES: fill_n
// constant assignment -  APL: A is con
//////////////////////////////////////////////////////////////////////

bool test_001( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  int x = 17, y = 42, z = 99;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::fill_n(g_vw, x, size);

  ctr.stop();
  double time1 = ctr.value();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    h[i] = x;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "001", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: iota
// sequence assignment -  APL: monadic-iota A
//////////////////////////////////////////////////////////////////////

bool test_002( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::iota(g_vw, 0);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    h[i] = i;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "002", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: generate_n, sequence
// value assignment -  APL: A is A mul iota B
//////////////////////////////////////////////////////////////////////

bool test_003( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int base = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate_n(g_vw, 0, size, step_wf(base));

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    h[i] = i+base;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "003", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: generate_n, block_sequence
// value assignment -  APL: A is A mul iota B
//////////////////////////////////////////////////////////////////////

bool test_004( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int rep = 49;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate_n(g_vw, 0, size, repeat_wf(base,rep));

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int rep_p1 = rep + 1;
  for (int i = base; i < size/rep_p1; i++) {
    int start = i*rep_p1;
    for (int j = 0; j<rep_p1; j++) {
      h[start+j] = j;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "004", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform unary
// apply unary function to each scalar -  APL: A scalar-monadic B
//////////////////////////////////////////////////////////////////////

bool test_005( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::transform(a_vw, g_vw, ineg_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    h[i] = -a[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "005", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform binary
// apply binary function to each scalar -  APL: A scalar-dyadic B
//////////////////////////////////////////////////////////////////////

bool test_006( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {


  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::transform(a_vw,b_vw, g_vw, imax_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    h[i] = std::max(a[i], b[i]);
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "006", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: accumulate
// arithmetic reduction -  APL: plus reduce A
//////////////////////////////////////////////////////////////////////

bool test_007( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int alg_int = stapl::accumulate(a_vw,0);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = 0;
  for (int i = 0; i < size; i++) {
    ref_int += a[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "007", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: max_value
// arithmetic reduction -  APL: max reduce A
//////////////////////////////////////////////////////////////////////

bool test_008( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int alg_int = stapl::max_value(b_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

#define max_val(x,y) ((x>y)?(x):(y))
  int ref_int = numeric_limits<int>::min();
  for (int i = 0; i < size; i++) {
    ref_int = max_val(ref_int,b[i]);
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "008", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: all_of
// logical reduction -  APL: and reduce A
//////////////////////////////////////////////////////////////////////

bool test_009( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  bool alg_bool = stapl::all_of(c_vw, iid_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  bool ref_bool = true;
  for (int i = 0; i < size; i++) {
    ref_bool = ref_bool & c[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_bool = alg_bool;
#endif

  stapl::rmi_fence();
  return check_scalar<bool>( "009", "STAPL", "manual",
                             time1, time1, 0, 0, alg_bool, ref_bool);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: max_element
// location reduction -  APL: A index max reduce A
//////////////////////////////////////////////////////////////////////

bool test_010( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::reference alg_ref = stapl::max_element(a_vw);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  int temp = numeric_limits<int>::min();
  for (int i = 0; i < size; i++) {
    if (temp < a[i] ) {
      temp = a[i];
      ref_int = i;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "010", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// arithmetic scan -  APL: plus scan A
//////////////////////////////////////////////////////////////////////

bool test_011( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int final = stapl::accumulate(a_vw, 0, iadd_wf());
  stapl::scan(a_vw, g_vw, iadd_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  h[0] = a[0];
  for (int i = 1; i < size; i++) {
    h[i] = a[i] + h[i-1];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "011", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// arithmetic scan -  APL min scan A
//////////////////////////////////////////////////////////////////////

bool test_012( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int final = stapl::accumulate(b_vw, 0, imin_wf());
  stapl::scan(b_vw, g_vw, imin_wf(), false);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  h[0] = b[0];
  for (int i = 1; i < size; i++) {
    h[i] = std::min(h[i-1], b[i]);
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "012", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: scan
// logical scan -  APL: or scan A
//////////////////////////////////////////////////////////////////////

bool test_013( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::scan(d_vw, g_vw, ior_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  h[0] = 1;
  for (int i = 1; i < size; i++) {
    h[i] = h[i-1] | d[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "013", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:
// invalid test deleted
//////////////////////////////////////////////////////////////////////

bool test_014( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

return true;
}

//////////////////////////////////////////////////////////////////////
// FEATURES: adjacent_difference
// adjacent delta - APL: 0 chain ((-1 drop A) sub (1 drop A)))
//////////////////////////////////////////////////////////////////////

bool test_015( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::adjacent_difference(b_vw, g_vw, isub_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  h[0] = b[0];
  for (int i = 1; i < size; i++) {
    h[i] = b[i] - b[i-1];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "015", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: inner_product
// inner product -  APL: A op1 innerprodd op2 B
//////////////////////////////////////////////////////////////////////

bool test_016( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int alg_int = stapl::inner_product(a_vw, b_vw, 0, iadd_wf(), imul_wf());

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = 0;
  for (int i = 0; i < size; i++) {
    ref_int += a[i] * b[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "016", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: STAPL doesn't have an outer product
// outer product -  APL: A jot dot op B
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// FEATURES: reverse_copy
// DEFECT - not being tested
// reverse -  APL: reverse A
//////////////////////////////////////////////////////////////////////

bool test_017( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::reverse_copy(a_vw, g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = size - 1;
  for (int i = 0; i < size; i++) {
    h[j] = a[i];
    --j;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "017", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: rotate_copy
// rotate elements -  APL: k rotate A
//////////////////////////////////////////////////////////////////////

bool test_018( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;
  stapl::rotate_copy(b_vw, g_vw, n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = b[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "018", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// take head -  APL: n take A
//////////////////////////////////////////////////////////////////////

bool test_019( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom);
  ary_int_view_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  ary_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  ary_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill_n(gpost_vw, -12345, size-n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i <=n-1; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i <=size-1; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "019", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// take tail -  APL: -n take A
//////////////////////////////////////////////////////////////////////

bool test_020( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpost_dom( size-n, size-1, b_dom);
  ary_int_view_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, n-1, g_dom);
  dom_tp gpost_dom( n, size-1, g_dom);
  ary_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  ary_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == n );

#if 1
  // why doesn't this work?
  stapl::copy_n(bpost_vw, gpre_vw, n);
#else
  stapl::copy(bpost_vw, gpre_vw);
#endif
  stapl::fill_n(gpost_vw, -12345, size-n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = size-n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = n; i < size; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "020", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// drop head -  APL: n drop A
//////////////////////////////////////////////////////////////////////

bool test_021( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, n-1, b_dom); // not used
  dom_tp bpost_dom( n, size-1, b_dom);
  ary_int_view_tp bpost_vw(b_vw.container(), bpost_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);

  ary_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  ary_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + (bpost_dom.last() - bpost_dom.first());
  int gcount = 1 + (gpre_dom.last() - gpre_dom.first());
  assert( bcount == gcount );
  assert( bcount == size - n );

#if 1
  // why doesn't this work?
  stapl::copy_n(bpost_vw, gpre_vw, size-n);
  stapl::fill_n(gpost_vw, -12345, n);
#else
  stapl::copy(bpost_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = n; i < size; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "021", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// drop tail -  APL: -n drop A
//////////////////////////////////////////////////////////////////////

bool test_022( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 97;

  // input domains and views
  dom_tp b_dom = b_vw.domain();
  dom_tp bpre_dom( 0, size-(n+1), b_dom);
  dom_tp bpost_dom( size-n, size-1, b_dom); // not used
  ary_int_view_tp bpre_vw(b_vw.container(), bpre_dom);

  // output domains and views
  dom_tp g_dom = g_vw.domain();
  dom_tp gpre_dom( 0, size-(n+1), g_dom);
  dom_tp gpost_dom( size-n, size-1, g_dom);
  ary_int_view_tp gpre_vw(g_vw.container(), gpre_dom);
  ary_int_view_tp gpost_vw(g_vw.container(), gpost_dom);

  int bcount = 1 + bpre_dom.last() - bpre_dom.first();
  int gcount = 1 + gpre_dom.last() - gpre_dom.first();
  assert( bcount == gcount );
  assert( gcount == size - n );

#if 1
  // why doesn't this work?
  stapl::copy_n(bpre_vw, gpre_vw, size-n);
  stapl::fill_n(gpost_vw, -12345, n);
#else
  stapl::copy(bpre_vw, gpre_vw);
  stapl::fill(gpost_vw, -12345);
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i < size-n; i++) {
    h[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    h[j++] = -12345;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "022", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// pad array at back -  APL: (size+n) pad A
//////////////////////////////////////////////////////////////////////

bool test_023( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 53;

  ary_int_tp za(size+n);
  ary_int_view_tp za_vw(za);

  dom_tp za_dom = za_vw.domain();
  dom_tp x_dom( 0, size-1, za_dom);
  dom_tp y_dom( size, size+n, za_dom);

  ary_int_view_tp x_vw(za_vw.container(), x_dom);
  ary_int_view_tp y_vw(za_vw.container(), y_dom);

#if 1
  // why doesn't this work?
  stapl::copy_n(a_vw, x_vw, size);
  stapl::fill_n(y_vw, 0, n);
#else
  stapl::copy(b_vw, x_vw);
  stapl::fill(y_vw, 0);
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  ary_int_tp zr(size+n);
  ary_int_view_tp zr_vw(zr);

  int j = 0;
  for (int i = 0; i < size; i++) {
    zr[j++] = b[i];
  }
  for (int i = 0; i <n; i++) {
    zr[j++] = 0;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "023", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// pad array at front -  APL: (-(size+n)) pad A
//////////////////////////////////////////////////////////////////////

bool test_024( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int n = 53;

  ary_int_tp za(size+n);
  ary_int_view_tp za_vw(za);
  ary_int_tp zr(size+n);
  ary_int_view_tp zr_vw(zr);

  dom_tp za_dom = za_vw.domain();
  dom_tp x_dom( 0, n-1, za_dom);
  dom_tp y_dom( n, size+n, za_dom);

  ary_int_view_tp x_vw(za_vw.container(), x_dom);
  ary_int_view_tp y_vw(za_vw.container(), y_dom);

#if 1
  // why doesn't this work?
  stapl::fill_n(x_vw, 0, n);
  stapl::copy_n(b_vw, y_vw, size);
#else
  stapl::fill(x_vw, 0);
  stapl::copy(b_vw, y_vw);
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i <n; i++) {
    zr[j++] = 0;
  }
  for (int i = 0; i < size; i++) {
    zr[j++] = b[i];
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "024", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n, fill_n
// DEFECT: need gather/scatter implementation
// expand elements -  APL: A expand B
//////////////////////////////////////////////////////////////////////

bool test_025( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  return known_fail( "025", "STAPL", "manual",
                     "stapl::gather() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::iterator::difference_type alg_int;
  alg_int = stapl::count_if(a_vw,
                              bind(stapl::equal_to<int>(),_1,0) );
  int n = (int)alg_int;

  ary_int_tp z(size+n);
  ary_int_view_tp z_vw(z);

  dom_tp b_dom = b_vw.domain();
  dom_tp x_dom( 0, size+n, b_dom);
  ary_int_view_tp x_vw(z_vw.container(), x_dom);

  stapl::fill_n(x_vw, 0, size+n);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i <n; i++) {
    if (a[i] == 1 ) {
      h[j++] = b[i];
    } else {
      h[j++] = 0;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "025", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}


//////////////////////////////////////////////////////////////////////
// FEATURES: domains, copy_n
// DEFECT: need gather/scatter implementation
// replicate -  APL: A repeat B
//////////////////////////////////////////////////////////////////////

bool test_026( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  return known_fail( "026", "STAPL", "manual",
                     "stapl::gather() feature required", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::transform(a_vw, g_vw, bind(stapl::minus<int>(),_1,1) );
  int n = stapl::accumulate(g_vw, 0);

  ary_int_tp x(size+n);
  ary_int_view_tp x_vw(x);

  dom_tp a_dom = a_vw.domain();
  dom_tp z_dom( size, size+n, a_dom);
  ary_int_view_tp z_vw(a_vw.container(), z_dom);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

// FIXME explicit domain = indirect array?

  int k = 0;
  for (int i = 0; i <n; i++) {
    int temp = a[i];
    for (int j = 0; j<temp; j++) {
      h[k++] = b[i];
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "026", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: copy, sort, sample_sort
// sorting operations -  APL: (gradeup A) pick A
//////////////////////////////////////////////////////////////////////

bool test_027( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::copy(a_vw, g_vw);
  stapl::sample_sort(g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  stapl::copy(a_vw, h_vw);
  stapl::sort(h_vw);

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "027", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: copy, sort, radix_sort
// sorting operations -  APL: (gradeup A) pick A
//////////////////////////////////////////////////////////////////////

bool test_028( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::copy(a_vw, g_vw);
  stapl::radix_sort(g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  stapl::copy(a_vw, h_vw);
  stapl::sort(h_vw);

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "028", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES:  STAPL does not have a grade algorithm
// list elements in order to pick for soriting - APL: gradeup A
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
// FEATURES: find
// search operations -  APL: A index B
//////////////////////////////////////////////////////////////////////

bool test_029( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/2];
  ary_int_view_tp::reference alg_ref = stapl::find(a_vw, x);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++ ) {
    if ( a[i] == x ) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "029", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: search
// search for subsequence -  APL: A search B
//////////////////////////////////////////////////////////////////////

bool test_030( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  // build data to be searched
  stapl::fill_n(p_vw, -1, size);

  int pat_len = 20;
  int k = 0;
  for (int i= 0; i < 10; i++ ) {

    // copy filler
    for (int j = 0; j< fibo20[i]; j++ ) {
      p[k++] = j;
    }

    // copy [part of] pattern
    for (int j = 0; j < pat_len; j++ ) {
      if (0 != (i+1)%10 && i == j ) {
        continue;
      }
      p[k++] = fibo20[j];
    }
  }

#if 0
  for (; k < 1000; ++k ) {
    p[k] = -1;
  }
#endif

  // build pattern

  dom_tp q_dom = q_vw.domain();
  dom_tp pat_dom( 0, pat_len-1, q_dom);
  ary_int_view_tp pat_vw(q_vw.container(), pat_dom);

  for (int j = 0; j < pat_len; j++ ) {
    q[j] = fibo20[j];
  }

  stapl::rmi_fence();

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::reference alg_ref = stapl::search(p_vw, pat_vw);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int count = pat_vw.domain().size();
  int ref_int = -1;
  int j = 0;
  for (int i = 0; i < size; ) {
    // find the anchor
    if ( p[i] == q[0] ) {
      bool found = true;
      for (j =1; j<count; j++ ) {
        if ( p[i+j] != q[j] ) {
          found = false;
          break;
        }
      }
      if (found) {
        ref_int = i;
        break;
      }
      i += j + 1;
    } else {
      i += 1;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "030", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: binary_search
// binary search -  APL: A member B *NONE*
//////////////////////////////////////////////////////////////////////

bool test_031( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = b[size/2];
  bool alg_bool = stapl::binary_search(b_vw, x);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  bool ref_bool = false;
  int left = 0;
  int right = size - 1;
  while ( right >= left ) {
    int mid = (left + right) / 2;
    if (b[mid] == x ) {
      ref_bool = true;
      break;
    }
    if (x < b[mid] ) {
      right = mid - 1;
    } else {
      left = mid + 1;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_bool = alg_bool;
#endif

  stapl::rmi_fence();
  return check_scalar<bool>( "031", "STAPL", "manual",
                             time1, time1, 0, 0, alg_bool, ref_bool);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: find_end
// search from end -   APL: (reverse A) search B
//////////////////////////////////////////////////////////////////////

bool test_032( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  // build data to be searched
  stapl::fill_n(p_vw, -1, size);

  int pat_len = 20;
#ifdef REWRITE_THIS
  int k = 0;
  for (int i= 0; i < 1; i++ ) {

    // copy filler
    for (int j = 0; j< fibo20[i]; j++ ) {
      p[k++] = j;
    }

    // copy pattern
    for (int j = 0; j < pat_len; j++ ) {
      p[k++] = fibo20[j];
    }
  }
#endif

#ifdef DELETE
  for (; k < 1000; ++k ) {
    p[k] = -1;
  }
#endif

  // build pattern

  dom_tp q_dom = q_vw.domain();
  dom_tp pat_dom( 0, pat_len-1, q_dom);
  ary_int_view_tp pat_vw(q_vw.container(), pat_dom);

  for (int j = 0; j < pat_len; j++ ) {
    q[j] = fibo20[j];
  }

  stapl::rmi_fence();

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::reference alg_ref = stapl::find_end(p_vw, pat_vw);

  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int count = pat_vw.domain().size();
  int ref_int = -1;
  int j = 0;
  for (int i = 0; i < size; ) {
    // find the anchor
    if ( p[i] == q[0] ) {
      bool found = true;
      for (j =1; j<count; j++ ) {
        if ( p[i+j] != q[j] ) {
          found = false;
          break;
        }
      }
      if (found) {
        ref_int = i;
      }
      i += j + 1;
    } else {
      i += 1;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "032", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: find_if
// APL: (pred A) index 1
//////////////////////////////////////////////////////////////////////

bool test_033( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::reference alg_ref = stapl::find_if(a_vw, even_value<int>());
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = 0;
  for (int i = 0; i < size; i++) {
    if ( 0 == a[i] % 2) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "033", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: find_first_of
// APL: min_reduce (A index B)
//////////////////////////////////////////////////////////////////////

bool test_034( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

#ifdef PERF_BUG
  ary_int_view_tp::reference alg_ref = stapl::find_first_of(a_vw, e_vw);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);
#else
  int alg_int = 0;
#endif

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = size;
  for (int i = 0; i < size; i++) {
    for (int j = 0; j< size; j++) {
      if ( a[i] == e[j] ) {
        if (i < ref_int ) {
          ref_int = i;
          break;
        }
      }
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "034", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: all_of
// APL: and reduce (pred A)
//////////////////////////////////////////////////////////////////////

bool test_035( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  bool alg_bool = stapl::all_of(b_vw, bind(stapl::less<int>(),_1,1000) );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  bool ref_bool = true;
  for (int i = 0; i < size; i++) {
    ref_bool &= b[i] < 1000;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  bool ref_bool = alg_bool;
#endif

  stapl::rmi_fence();
  return check_scalar<bool>( "035", "STAPL", "manual",
                             time1, time1, 0, 0, alg_bool, ref_bool);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: mismatch
// APL: (A neq B) index 0
//////////////////////////////////////////////////////////////////////

bool test_036( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  return known_fail( "036", "STAPL", "manual",
                     "stapl::mismatch() failure", 0 );

  stapl::copy(a_vw, g_vw);
  g[10] = 1024;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  typedef ary_int_view_tp::reference ref1;
  typedef ary_int_view_tp::reference ref2;
  std::pair<ref1, ref2> values = make_pair(ref1(stapl::null_reference()),
                                           ref2(stapl::null_reference()) );

  values = stapl::mismatch(a_vw, g_vw, ieq_wf());

  if (is_null_reference(values.first) ) {
    return false;
  }
  int alg_int = index_of(values.first);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++) {
    if (a[i] != b[i] ) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  bool ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "036", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: equal
// APL: A match B
//////////////////////////////////////////////////////////////////////

bool test_037( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  bool alg_bool = stapl::equal(a_vw, b_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  bool ref_bool = true;
  for (int i = 0; i < size; i++) {
    if (a[i] != b[i] ) {
      ref_bool = false;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  bool ref_bool = alg_bool;
#endif

  stapl::rmi_fence();
  return check_scalar<bool>( "037", "STAPL", "manual",
                             time1, time1, 0, 0, alg_bool, ref_bool);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: transform, scan, copy_if
// select true indices -   APL: (pred) compress iota B
//////////////////////////////////////////////////////////////////////

bool test_038( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  return known_fail( "038", "STAPL", "manual",
                     "stapl functionality issue", 0 );

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::iota(p_vw, 0);
  stapl::copy_if(p_vw, g_vw, bind(less<int>(),_1,500) );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i < size; i++) {
    if ( p[i] < 500 ) {
      h[j++] = i;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "038", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: remove_copy
// APL: (A neq x ) compress A
//////////////////////////////////////////////////////////////////////

bool test_039( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/2];
  stapl::remove_copy(a_vw, g_vw, x);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i < size; i++) {
    if (a[i] != x ) {
      h[j++] = a[i];
    }
  }
  for (; j< size; j++) {
    h[j] = x;
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "039", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: remove_copy_if
// APL: (pred) compress A
//////////////////////////////////////////////////////////////////////

bool test_040( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::remove_copy_if(f_vw, g_vw, bind(stapl::less<int>(),_1,500) );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int j = 0;
  for (int i = 0; i < size; i++) {
    if ( !(f[i] < 500) ) {
      h[j++] = f[i];
    }
  }
  for (int i = 0; i < size; i++) {
    if ( (f[i] < 500) ) {
      h[j++] = f[i];
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "040", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: unique_copy
// APL: (A neq 1 rotate A) compress A
//////////////////////////////////////////////////////////////////////

bool test_041( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::copy(e_vw, p_vw);
  stapl::unique_copy(p_vw, g_vw);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  stapl::copy(e_vw, q_vw);

  int k = 0;
  for (int i = 1; i < size; i++) {
    if (q[i] == q[i-1] ) {
      k++;
    }
  }

  int j = 0;
  h[j++] = q[0];
  for (int i = 1; i < size; i++) {
    if (q[i] != q[i-1] ) {
      h[j++] = q[i];
    } else {
      h[size-k] = q[i];
      k--;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "041", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: replace_copy
// APL: A [ (A[i] eq X) compress iota shape A ] is Y
//////////////////////////////////////////////////////////////////////

bool test_042( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  int x = 17, y = 42, z = 99;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::replace_copy(a_vw, g_vw, x, y);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    if (a[i] == x ) {
      h[i] = y;
    } else {
      h[i] = a[i];
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "042", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: replace_copy_if
// APL: A [ (pred) compress iota shape A ] is X
//////////////////////////////////////////////////////////////////////

bool test_043( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  int x = 17, y = 42, z = 99;

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  typedef odd_value<int> odd_wf;
  stapl::replace_copy_if(a_vw, g_vw, odd_wf(), x);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  for (int i = 0; i < size; i++) {
    if ( 1 == a[i] % 2 ) {
      h[i] = x;
    } else {
      h[i] = a[i];
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#endif

  stapl::rmi_fence();
  return check_container<ary_int_tp>( "043", "STAPL", "manual",
                                  time1, time1, 0, 0, g, g );
}

//////////////////////////////////////////////////////////////////////
// FEATURES: count
// APL: sum reduce (A eq B)
//////////////////////////////////////////////////////////////////////

bool test_044( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/4];
  ary_int_view_tp::iterator::difference_type alg_int;
  alg_int= stapl::count(a_vw, x);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = 0;
  for (int i = 0; i < size; i++) {
    if (a[i] == x ) {
      ref_int++;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "044", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: count_if
// APL: sum reduce (pred A)
//////////////////////////////////////////////////////////////////////

bool test_045( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  ary_int_view_tp::iterator::difference_type alg_int;
  alg_int = stapl::count_if(a_vw, bind(stapl::less<int>(),_1,500) );

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = 0;
  for (int i = 0; i < size; i++) {
    if (a[i] < 500 ) {
      ref_int++;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "045", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: adjacent_find
// APL: (A equal 1 rotate A) index 1
//////////////////////////////////////////////////////////////////////

bool test_046( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  stapl::copy(a_vw,p_vw);
  p[25] = p[24];
  p[50] = p[49];
  p[75] = p[74];
  p[100] = p[99];

  ary_int_view_tp::reference alg_ref = stapl::adjacent_find(p_vw);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 1; i < size; i++) {
    if (p[i] == p[i-1] ) {
      ref_int = i-1;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "046", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: partition_copy
// APL: *IDIOM*
//////////////////////////////////////////////////////////////////////

bool test_047( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  typedef pair<ary_int_view_tp,ary_int_view_tp> o_vw_pair;
  (void)stapl::partition_copy(a_vw, g_vw, h_vw,
                                    bind(stapl::less<int>(),_1,500) );
  int alg_int = 0;

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++) {
    if (h[i] >= 500 ) {
      ref_int = i;
      alg_int = i; // wierdness to make checking work
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "047", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: lower_bound
// APL: *IDIOM*
//////////////////////////////////////////////////////////////////////

bool test_048( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/8];
  ary_int_view_tp::reference alg_ref = stapl::lower_bound(a_vw, x);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++) {
    if ( a[i] >= x ) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "048", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: upper_bound
// APL: *IDIOM*
//////////////////////////////////////////////////////////////////////

bool test_049( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/8];
  ary_int_view_tp::reference alg_ref = stapl::upper_bound(a_vw, x);
  if (is_null_reference(alg_ref) ) {
    return false;
  }
  int alg_int = index_of(alg_ref);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  int ref_int = -1;
  for (int i = 0; i < size; i++) {
    if ( a[i] > x ) {
      ref_int = i;
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
#else
  int ref_int = alg_int;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "049", "STAPL", "manual",
                            time1, time1, 0, 0, alg_int, ref_int);
}

//////////////////////////////////////////////////////////////////////
// FEATURES: equal_range
// APL:  (A eq B) compress B
//////////////////////////////////////////////////////////////////////

bool test_050( ary_int_tp &a, ary_int_tp &b, ary_int_tp &c, ary_int_tp &d,
                  ary_int_tp &e, ary_int_tp &f, ary_int_tp &g, ary_int_tp &h,
                  ary_int_tp &p, ary_int_tp &q,
                   ary_int_view_tp& a_vw, ary_int_view_tp& b_vw,
                   ary_int_view_tp& c_vw, ary_int_view_tp& d_vw,
                   ary_int_view_tp& e_vw, ary_int_view_tp& f_vw,
                   ary_int_view_tp& g_vw, ary_int_view_tp& h_vw,
                   ary_int_view_tp& p_vw, ary_int_view_tp& q_vw, int size) {

  stapl::counter<stapl::default_timer> ctr;
  ctr.start();

  int x = a[size/8];
  ary_int_view_tp o_vw = stapl::equal_range(a_vw, x);

  ctr.stop();
  double time1 = ctr.value();
  ctr.reset();

#ifdef REFERENCE
  ctr.start();

  size_t first = std::numeric_limits<size_t>::max();
  size_t  last = std::numeric_limits<size_t>::max();
  for ( int i = 0; i < size; i++) {
    if ( a[i] == x ) {
      first = i;
      for ( int j = i; j < size; j++ ) {
        if (a[j] != x ) {
          last = j - 1;
          break;
        }
      }
      break;
    }
  }

  ctr.stop();
  double time2 = ctr.value();
  ctr.reset();
  bool check1 = first == o_vw.domain().first();

  bool check2 = last == o_vw.domain().last();
#else
  bool check1 = true, check2 = true;
#endif

  stapl::rmi_fence();
  return check_scalar<int>( "050", "STAPL", "manual",
                            time1, time1, 0, 0, check1, check2);
}
