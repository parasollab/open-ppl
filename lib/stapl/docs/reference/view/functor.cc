/* ****************************** NOT DONE ****************************** */
// view/functor.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include <boost/random.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "viewhelp.hpp"
using namespace std;

template <typename T>
struct const_gener
{
private:
  T m_value;
public:
  typedef std::size_t index_tp;
  const_gener(T value)
    : m_value(value)
  { }

  typedef T result_type;
  result_type operator()(index_tp const& idx) const
  { return m_value; }

  void define_type(stapl::typer& t)
  { t.member(m_value); }
};

template <typename T>
struct rand_gener
{
private:
  T m_value;
  std::size_t m_seed;
public:
  typedef std::size_t index_tp;
  rand_gener(T seed)
    : m_seed(seed)
  { }

  typedef T result_type;
  result_type operator()(index_tp const& idx) const
  {
    typedef boost::mt19937 engine_tp;
    typedef boost::random::uniform_real_distribution<double> dist_tp;
    typedef boost::variate_generator<engine_tp, dist_tp> gener_tp;

    engine_tp eng(idx*m_seed + idx);
    gener_tp pos_gen(eng, dist_tp(0,1));
    return pos_gen();
  }

  void define_type(stapl::typer& t)
  { t.member(m_seed); }
};

typedef int val_tp;
typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

typedef stapl::plus<val_tp> add_int_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  ulong count = 40;
  // construct container and view over container
  ary_int_tp a_ct(count), b_ct(count), c_ct(count);
  ary_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  // initialize container
  stapl::iota(a_vw, 0);

  // construct functor views
  val_tp param = 28128;
#if 0
  auto fn_con = stapl::functor_view(count, const_gener<ulong>(param));
  auto fn_rand = stapl::functor_view(count, rand_gener<ulong>(16807));
#endif

  // process elements through view
#if 0
  stapl::map_func( id_int_wf(), add_int_wf(), fn_con, a_vw, b_vw );
  stapl::map_func( id_int_wf(), add_int_wf(), rand_con, a_vw, c_vw );
#endif

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_funcvw.txt");

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */
