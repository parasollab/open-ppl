/*
// Copyright (c) 2000-2015, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <iostream>
#include <fstream>
#include <cstdlib>

#include <stapl/utility/do_once.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/stream.hpp>

#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;

typedef stapl::vector<double> vec_dbl_tp;
typedef stapl::vector_view<vec_dbl_tp> vec_dbl_vw_tp;

// functor used to read values from file in parallel

struct get_val_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  get_val_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;

  template<typename Ref>
  result_type operator() (Ref val)
  { m_zin >> val; }

  void define_type(stapl::typer& t)
  { t.member(m_zin); }
};

stapl::exit_code stapl_main(int argc, char ** argv)
{
  // validate command line arguments
  if ( argc < 2 ) {
    stapl::do_once( [&]() {
      cout << "Must specify input file name" << endl;
    } );
    return EXIT_FAILURE;
  }
  stapl::stream<ifstream> zin;
  zin.open(argv[1]);

  if ( argc < 3 ) {
    stapl::do_once( [&]() {
      cout << "Must specify input data count" << endl;
    } );
    return EXIT_FAILURE;
  }
  int count = atoi(argv[2]);
  if ( count <= 0 ) {
    stapl::do_once( [&]() {
      cout << "Not a valid input data count: " << argv[2] << endl;
    } );
    return EXIT_FAILURE;
  }

  // read input from file
  vec_dbl_tp data(count), temp(count), diff(count);
  vec_dbl_vw_tp data_vw(data), temp_vw(temp), diff_vw(diff);

  stapl::serial_io(get_val_wf(zin), data_vw);

  // compute statistics
  typedef stapl::min<double> min_dbl_wf;
  typedef stapl::max<double> max_dbl_wf;
  typedef stapl::plus<double> add_dbl_wf;

  double min = stapl::reduce( data_vw, min_dbl_wf());
  double max = stapl::reduce( data_vw, max_dbl_wf());
  double tot = stapl::reduce( data_vw, add_dbl_wf());
  double mean = tot / count;

  // median
  stapl::copy(data_vw, temp_vw);
  stapl::sort(temp_vw);

  double med = 0.0;
  int ndx = temp.size()/2;
  if ( 0 == temp.size() % 2 ) {
    med = 0.5 * (temp_vw[ndx-1] + temp_vw[ndx]);
  } else {
    med = temp_vw[ ndx + 1 ];
  }

  // standard deviation
  typedef stapl::minus<double> sub_dbl_wf;
  typedef stapl::multiplies<double> mul_dbl_wf;
  auto mean_vw = stapl::make_repeat_view(mean);
  stapl::transform(data_vw, mean_vw, diff_vw, sub_dbl_wf() );
  stapl::transform(diff_vw, diff_vw, temp_vw, mul_dbl_wf() );
  double var = stapl::reduce( temp_vw, add_dbl_wf());
  var /= count - 1;
  double sdev = sqrt(var);

  // display results
  stapl::do_once( [&]() {
    cout << "Descriptive statistics N= " << count << endl;
    cout << "min     " << min << "  ";
    cout << "max     " << max << "  ";
    cout << "mean    " << mean << "  ";
    cout << "median  " << med << "  ";
    cout << "std dev " << sdev << endl;
  } );

  return EXIT_SUCCESS;
}
