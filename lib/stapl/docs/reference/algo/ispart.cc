// algo/ispart.cc

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
struct half_pred
{
private:
  T m_mid;
public:
  typedef T    argument_type;
  half_pred(T mid) :
    m_mid(mid)
  { }
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return x < m_mid;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_mid);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  // apply algorithm
  val_tp mid = v_vw[v_vw.size()/2];
  bool test = stapl::is_partitioned(v_vw, half_pred<val_tp>(mid));

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_ispart.txt");
  stapl::do_once( msg_val<bool>( zout, "test ", test ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}
