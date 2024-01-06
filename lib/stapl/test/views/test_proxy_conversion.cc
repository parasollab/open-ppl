#include <stapl/vector.hpp>
#include <stapl/algorithm.hpp>

// Functor receives two proxies over numeric values and multiplies them.
struct binary_val_wf
{
  typedef double result_type;

  template<typename Ref1, typename Ref2>
  double operator() (Ref1 x, Ref2 y)
  { return x*y; }
};

typedef stapl::vector<int>                vec_int_tp;
typedef stapl::vector<double>             vec_double_tp;
typedef stapl::vector_view<vec_double_tp> vec_double_vw_tp;
typedef stapl::vector_view<vec_int_tp>    vec_int_vw_tp;

stapl::exit_code stapl_main(int, char**)
{
  size_t sz(1024);

  // Initialize containers so the expected result is 4.4 * 1024 == 4505.6
  vec_int_tp a(sz, 2);
  vec_double_tp b(sz, 2.2), c(sz, 1.);

  vec_int_vw_tp a_vw(a);
  vec_double_vw_tp b_vw(b), c_vw(c);

  stapl::transform(a_vw, b_vw, c_vw, binary_val_wf());

  double result = stapl::accumulate(c_vw, 0.);

  stapl::do_once([result]{
    double lower = 2*2.2*1024 - 0.000001;
    double upper = 2*2.2*1024 + 0.000001;
    if (lower >= result || result >= upper)
      std::cout << "conversion in proxy operation failed.\n";
    else
      std::cout << "conversion in proxy operation passed.\n";
  });

  return EXIT_SUCCESS;
}
