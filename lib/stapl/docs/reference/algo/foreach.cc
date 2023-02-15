// algo/foreach.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

template <typename Functor>
struct self_assign
{
  typedef void result_type;

  Functor m_func;

  self_assign(Functor const& f)
    : m_func(f)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_func);
  }

  template <typename Ref>
  void operator()(Ref val)
  {
    val = m_func(val);
  }
};

template <typename Functor>
self_assign<Functor> make_self_assign(Functor const& f)
{
  return self_assign<Functor>(f);
}

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(30);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  // apply algorithm
  stapl::for_each(v_vw,
         make_self_assign( bind(stapl::plus<val_tp>(),_1,1000)) );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_foreach.txt");

  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}
