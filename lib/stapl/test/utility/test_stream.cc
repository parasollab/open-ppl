/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <fstream>

#include <stapl/vector.hpp>
#include <stapl/numeric.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>

using namespace std;
#include "../rel_alpha/rel_alpha_util.hpp"

/*=========================================================================*/

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view< vec_int_tp > vec_int_vw_tp;

/*=========================================================================*/

size_t test_streams( size_t,
                     stapl::stream<ifstream> &, stapl::stream<ofstream> &,
                     bool use_getline = false );

/*=========================================================================*/

struct reflect_local_value
  : public stapl::p_object
{
  int reflect(int val)
  { return val; }
};

struct write_test_wf
{
private:
  stapl::stream<ofstream> m_zout;

public:
  typedef void result_type;

  write_test_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Value>
  result_type operator()(Value v)
  {
    int val = v;
    char buf[4];
    std::memcpy(buf, &val, sizeof(val));
    m_zout.write(buf, 4);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


stapl::exit_code stapl_main(int argc, char **argv)
{
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;

  zout.open("tiny_factors.zout");
  zin.open("../rel_alpha/tiny_factors.zin");

  size_t num_lines = 1958;

  // Expected result obtained with
  // cat tiny_factors.zin | awk '{s+=$1; s+=$2; s+=$3} END {print s}'
  const size_t expected_res = 1095915;
  size_t res = test_streams(num_lines, zin, zout);

  stapl::do_once([=](void) {
    if (expected_res == res)
      std::cerr << "stapl::stream test PASSED\n";
    else
      std::cerr << "stapl::stream test FAILED\n";
  });

  // Test getline
  zin.close();
  zout.close();
  zin.open("../rel_alpha/tiny_factors.zin");
  zout.open("tiny_factors_getline.zout");
  res = test_streams(num_lines, zin, zout, true);

  stapl::do_once([=](void) {
    if (expected_res == res)
      std::cerr << "stapl::stream::getline test PASSED\n";
    else
      std::cerr << "stapl::stream::getline test FAILED\n";
  });

  // Test read
  zin.close();
  zout.close();
  zin.open("sequence.bin");
  zout.open("sequence.new",std::ios_base::binary);

  reflect_local_value reflector;
  int local_sum = 0;
  char buf[4];
  int nelems_to_read = 1000 / stapl::get_num_locations();
  if (stapl::get_location_id() < 1000 % stapl::get_num_locations())
    ++nelems_to_read;

  for (int i=0; i != nelems_to_read; ++i)
  {
    zin.read(buf, 4);
    int tmp = 0;
    std::memcpy(&tmp, buf, sizeof(tmp));
    local_sum += tmp;
  }
  stapl::rmi_fence();

  stapl::future<int> global_sum =
    stapl::allreduce_rmi(stapl::plus<int>(), reflector.get_rmi_handle(),
      &reflect_local_value::reflect, local_sum);

  if (stapl::get_location_id() == 0)
  {
    int val = global_sum.get();
    if (val == 500500) // sum == 1000 * 1001 / 2
      std::cerr << "stapl::stream::read test PASSED\n";
    else
      std::cerr << "stapl::stream::read test FAILED\n";
  }
  else
  {
    global_sum.get();
  }

  stapl::vector<int> write_test_vector(1000);
  stapl::vector_view<stapl::vector<int>> write_test_view(write_test_vector);
  stapl::iota(write_test_view, 1);

  stapl::serial(write_test_wf(zout), write_test_view);

  return EXIT_SUCCESS;
}


struct put_triple_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  put_triple_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 first, Ref2 second, Ref3 third)
  {
    m_zout << first << " " << second << " " << third << std::endl;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};


struct getline_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  typedef void result_type;

  getline_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  template <typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 first, Ref2 second, Ref3 third)
  {
    char buf[10];
    m_zin.getline(buf, 10);

    // The line retrieved should be formatted as "x y z"
    first = atoi(buf);

    char* pos = std::find(buf, buf+10, ' ');
    ++pos;
    second = atoi(pos);

    pos = std::find(pos, buf+10, ' ');
    ++pos;
    third = atoi(pos);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

size_t test_streams( const size_t num_lines,
                     stapl::stream<ifstream> & zin,
                     stapl::stream<ofstream> & zout,
                     bool use_getline)
{
  vec_int_tp numbers(num_lines, 0);
  vec_int_vw_tp numbers_vw(numbers);

  vec_int_tp factors(num_lines, 0);
  vec_int_vw_tp factors_vw(factors);

  vec_int_tp counts(num_lines, 0);
  vec_int_vw_tp counts_vw(counts);

  if (!use_getline)
  {
    // get_triple_wf demonstrates chained uses of operator>>.
    stapl::serial_io(get_triple_wf(zin), numbers_vw, factors_vw, counts_vw);
  }
  else
    stapl::serial_io(getline_wf(zin), numbers_vw, factors_vw, counts_vw);

  // put_triple_wf demonstrates chained uses of operator<< and std::endl.
  stapl::serial_io(put_triple_wf(zout), numbers_vw, factors_vw, counts_vw);

  size_t result = stapl::accumulate(numbers_vw, 0);
  result += stapl::accumulate(factors_vw, 0);
  result += stapl::accumulate(counts_vw, 0);
  return result;
}
