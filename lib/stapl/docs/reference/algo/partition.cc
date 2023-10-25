// algo/stpart.cc

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

template <typename T>
struct pivot_pred
{
private:
  T m_piv;
public:
  typedef T    argument_type;
  pivot_pred(T piv) :
    m_piv(piv)
  { }
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return x < m_piv;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_piv);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers and views
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize containers
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  stapl::generate(v_vw, stapl::random_sequence());

  // apply algorithm
  val_tp piv = v_vw[v_vw.size() * .75];
  stapl::partition_copy(u_vw, v_vw, w_vw, pivot_pred<val_tp>(piv) );
  stapl::partition(u_vw, pivot_pred<val_tp>(piv) );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_partition.txt");

  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "w_vw" ) );
  stapl::serial_io( put_val_wf(zout), w_vw );
  stapl::do_once( msg( zout, "u_vw" ) );
  stapl::serial_io( put_val_wf(zout), u_vw );

  return EXIT_SUCCESS;
}
