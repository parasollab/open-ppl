// algo/randshuf.cc

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

template <typename Difference>
class rand_num_gen
{
  typedef Difference result_type;
  typedef boost::random::uniform_int_distribution<Difference> distrib_type;
  boost::random::mt19937 m_gen;
public:
  rand_num_gen(unsigned int seed)
    : m_gen(seed)
  { }

  Difference operator()(void)
  {
    return distrib_type()(m_gen);
  }

  Difference operator()(Difference n)
  {
    return distrib_type(0, n)(m_gen);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_gen);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(w_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(u_vw, repeat_wf(base,rep));

  // apply algorithm
  stapl::random_shuffle(u_vw);
  stapl::random_shuffle(v_vw, rand_num_gen<val_tp>(16807) );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_randshuf.txt");

  stapl::do_once( msg( zout, "v_vw") );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "u_vw") );
  stapl::serial_io( put_val_wf(zout), u_vw );
  stapl::do_once( msg( zout, "w_vw") );
  stapl::serial_io( put_val_wf(zout), w_vw );

  return EXIT_SUCCESS;
}
