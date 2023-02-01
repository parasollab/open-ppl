/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iomanip>
#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/map_reduce.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>
#include <stapl/skeletons/transformations/nest.hpp>
#include <stapl/skeletons/map.hpp>
#include "../test_report.hpp"

using namespace stapl;
using namespace stapl::skeletons;

namespace {

struct init_wf
{
  typedef void result_type;

  template <typename View, typename Index>
  result_type operator()(View const& vw, Index i) const
  {
    generate(vw, sequence<int>(i*10+4, 1));
  }
};

template <typename Op>
struct sum_wf
{
private:
  Op m_op;

public:
  typedef typename Op::result_type result_type;

  sum_wf(Op const& op)
    : m_op(op)
  { }

  template <typename V>
  result_type operator()(V&& v) const
  {
    return stapl::reduce(std::forward<V>(v), m_op);
  }

  void define_type(typer& t)
  {
    t.member(m_op);
  }
};

template <typename T>
struct increment
{
private:
  T m_i;
public:
  typedef void result_type;

  increment(T const& i)
    : m_i(i)
  { }

  template <typename E>
  void operator()(E&& e)
  {
    e += m_i;
  }

  void define_type(typer& t)
  {
    t.member(m_i);
  }
};

struct init_me
{
  typedef void result_type;

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    v1 = v2;
  }
};

} // namespace

stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "usage: exe n m" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);
  const size_t m = atoi(argv[2]);

  typedef std::size_t                value_type;
  typedef array<array<value_type>>   array_type;

  // Inner container distributed across the system
  array<value_type>                  c0(m);
  array_type                         c(n,c0);
  array_view<array_type>             vc(c);


  // Initialize the containers.
  map_func(init_wf(), vc, counting_view<value_type> (n, value_type(0)));

  //compute the reduction
  typedef stapl::plus<value_type> op_type;
  op_type         reduce_op;
  sum_wf<op_type> map_op(reduce_op);

  value_type r = stapl::map_reduce(map_op, reduce_op, vc);

  using skeletons::map;

  auto s1 = compose(
              skeletons::map_reduce(reduce(reduce_op), reduce_op),
              broadcast_to_locs<true>()
            );

  auto s2 = compose(
              map(reduce(reduce_op)),
              reduce_to_locs<true>(reduce_op)
            );

  auto s3 = compose(
              map(skeletons::map_reduce(
                    stapl::identity<value_type>(), reduce_op)),
              reduce_to_locs<true>(reduce_op)
            );


  value_type r1 = skeletons::execute(
                    skeletons::execution_params<value_type>(), s1, vc);
  value_type r2 = skeletons::execute(
                    skeletons::execution_params<value_type>(), s2, vc);
  value_type r3 = skeletons::execute(
                    skeletons::execution_params<value_type>(), s3, vc);

  // increment each value in the nested container by 10
  skeletons::execute(
    skeletons::default_execution_params(),
    map(map(increment<value_type>(10))),
    vc);

  value_type r4 = skeletons::execute(
                    skeletons::execution_params<value_type>(), s1, vc);

  STAPL_TEST_REPORT(
    r == r1, "Testing map-reduce(reduce(op), op).broadcast");
  STAPL_TEST_REPORT(
    r == r2, "Testing map(reduce(op)).reduce(op)");
  STAPL_TEST_REPORT(
    r == r3, "Testing map(map-reduce(id, reduce(op)).reduce(op) ");
  STAPL_TEST_REPORT(
    (r4 - r) == 10 * (m * n),
    "Testing map(map(increment) and map(coarse(map(decrement)) ");

  return EXIT_SUCCESS;
}
