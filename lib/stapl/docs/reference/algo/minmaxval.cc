/* ****************************** NOT DONE ****************************** */
// algo/minmaxval.cc

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

typedef pair<val_tp,val_tp> pair_val_tp;

template <typename T>
struct minmax_pred
{
private:
    val_tp m_min_temp;
    val_tp m_max_temp;
public:
  typedef T    argument_type;
  typedef pair_val_tp result_type;
  result_type operator() (const T& x) const
  {
    val_tp min_val = std::min( m_min_temp, x );
    val_tp max_val = std::max( m_max_temp, x );
    return pair_val_tp(min_val,max_val);
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_min_temp);
    t.member(m_max_temp);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  stapl::generate(v_vw, stapl::random_sequence());

  // apply algorithm
  pair_val_tp extrema;
#if 0
  extrema = stapl::minmax_value(v_vw, minmax_pred<pair_val_tp>() );
#endif

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_minmaxval.txt");
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "minval ", extrema.first ) );
  stapl::do_once( msg_val<val_tp>( zout, "maxval ", extrema.second ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
