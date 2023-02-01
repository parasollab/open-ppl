// view/stencil.cc

#include <stapl/array.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/stencil_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::multiarray<2, val_tp>      grid_tp;
typedef stapl::multiarray_view<grid_tp>   view_tp;

//////////////////////////////////////////////////////////////////////
/// Return different coefficients based on
/// how far a certain point is from the initial point.
//////////////////////////////////////////////////////////////////////
struct coefficient_view
{
  const double c0, c1, c2;

  coefficient_view(double c0_, double c1_, double c2_)
    : c0(c0_), c1(c1_), c2(c2_)
  { }

  double const& operator()(int i, int j) const
  {
    const int distance = (i != 0 ? 1 : 0) + (j != 0 ? 1 : 0);

    switch (distance)
    {
      case 0: return c0;
      case 1: return c1;
      default: return c2;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(c0);
    t.member(c1);
    t.member(c2);
  }
};

//////////////////////////////////////////////////////////////////////
/// Stencil work function that receives a single point and
/// the 9-point stencil view of its neighbors.
///
/// This computes a weighted average, where the weights for the points
//  with Manhattan distance of 0, 1 and 2 can be specified in the constructor.
//////////////////////////////////////////////////////////////////////

struct stencil9
{
private:
  coefficient_view m_coeff;
public:
  stencil9(double const c0, double const c1, double const c2)
    : m_coeff(c0, c1, c2)
  { }

  typedef void result_type;
  template<typename T, typename View>
  result_type operator()(T x, View vw)
  {
    auto const first = vw.domain().first();
    auto const last = vw.domain().last();

    for (std::size_t i = get<0>(first); i <= get<0>(last); ++i)
      for (std::size_t j = get<1>(first); j <= get<1>(last); ++j)
      {
        // get coefficient based on distance
        const int x_off = static_cast<int>(i)-get<0>(first)-1;
        const int y_off = static_cast<int>(j)-get<1>(first)-1;
        const double c = m_coeff(x_off, y_off);

        // add contribution of this point
        x += c * vw[make_tuple(i,j)];
      }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coeff);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {
  const std::size_t n = atol(argv[1]);

  // construct containers
  grid_tp lhs_grid(make_tuple(n, n));
  grid_tp rhs_grid(make_tuple(n, n));

  // construct views
  view_tp lhs(lhs_grid);
  view_tp rhs(rhs_grid);

  // initialize container:
  // fill grids with all zeros except for the first value, which is one
  fill(lhs, 0);
  fill(rhs, 0);

  stapl::do_once([&](){
    rhs[make_tuple(0, 0)] = 1;
  });

  // create view for stencil
  auto stencil = stapl::make_stencil_view<2>(rhs);

  // process elements through view
  // compute 9-point stencil
  map_func(stencil9(0.5, 0.25, 0.125), lhs, stencil);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_stnlvw.txt");
  stapl::serial_io( put_val_wf(zout), rhs );

  return EXIT_SUCCESS;
}
