/* ****************************** NOT DONE ****************************** */
// view/band.cc

#include <stapl/containers/matrix/matrix.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/matrix_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/band_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef double val_tp;
typedef stapl::multiarray<2,val_tp>       mat_dbl_tp;
typedef stapl::matrix_view<mat_dbl_tp>    mat_dbl_vw_tp;

typedef stapl::plus<val_tp> add_dbl_tp;
typedef stapl::max<val_tp>  max_dbl_tp;

struct fill_matrix_wf
{
private:
  int m_left_value;
  int m_diag_value;
  int m_right_value;
public:
  fill_matrix_wf(int lv, int diag, int rv)
    : m_left_value(lv), m_diag_value(diag), m_right_value(rv)
  { }

  typedef void result_type;
  template <typename T>
  result_type operator()(T m)
  {
    auto first_point = m.domain().first();

    size_t n = get<1>(m.domain().dimensions());
    size_t i = get<0>(first_point);
    size_t j = get<1>(first_point);

    if (i==0) {
      m(i,j++) = m_diag_value;
      m(i,j) = m_right_value;
    }
    if (n==2 && i>0) {
      m(i,j++) = m_left_value;
      m(i,j) = m_diag_value;
    }
    if (n==3) {
      m(i,j++) = m_left_value;
      m(i,j++) = m_diag_value;
      m(i,j)   = m_right_value;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_left_value);
    t.member(m_diag_value);
    t.member(m_right_value);
  }
};


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t n = 20;
  // construct container
  typedef typename mat_dbl_tp::dimensions_type  dim_tp;
  dim_tp dims = dim_tp(n,n);
  mat_dbl_tp m_ct(dims);

  // construct view over container
  auto mat_vw = mat_dbl_vw_tp(m_ct);
  auto band_vw = stapl::make_band_view(mat_vw,1,1);

  // initialize container
  stapl::map_func( fill_matrix_wf(-1,2,-1), band_vw);

  // process elements through view
  val_tp sum = stapl::map_reduce(add_dbl_tp(), max_dbl_tp(), a_vw, b_ext_vw);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_bandvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
