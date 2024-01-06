// view/zip.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/zip_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

class put_triple_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_triple_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }
  typedef void result_type;
  template <typename Arg>
  result_type operator()(Arg arg) {
    std::tuple<int,int,int> tup; // = arg // bug
    m_zout << get<0>(tup) << "," << get<1>(tup) << "," << get<2>(tup) << endl;
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

typedef int val_tp;
typedef stapl::vector<val_tp>               vec_int_tp;
typedef stapl::vector_view<vec_int_tp>      vec_int_vw_tp;
typedef stapl::zip_view<vec_int_vw_tp, vec_int_vw_tp, vec_int_vw_tp>
                                            zip3_vec_int_vw_tp;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 40;
  // construct container
  vec_int_tp a_ct(sz), b_ct(sz), c_ct(sz);

  // construct view over container
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  int rep = 42;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  stapl::generate(c_vw, stapl::random_sequence(1000));

  // process elements through view
  zip3_vec_int_vw_tp z_vw = zip3_vec_int_vw_tp(a_vw, b_vw, c_vw);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_zipvw.txt");
  stapl::do_once( msg( zout, "z:" ) );
  stapl::serial_io( put_triple_wf(zout), z_vw );

  return EXIT_SUCCESS;
}
