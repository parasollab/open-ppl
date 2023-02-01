#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>

#include <ctime>
#include <cstdio>
#include <string>
#include <cstdlib>

#include <iostream>
#include <fstream>
#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/tuple.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>

#include <boost/random/uniform_int_distribution.hpp>
#include <stapl/utility/random.hpp>


typedef stapl::bit_xor<size_t>  xor_un_wf;
typedef std::vector<std::size_t> std_cont;

void print_results(const char *name, stapl::tuple<bool, double> fin_res,
  int num_runs)
{
  bool passed = stapl::get<0>(fin_res);
  double exec_time = stapl::get<1>(fin_res) / num_runs;
  stapl::do_once([&](){
    std::cerr << "Test: " << name << std::endl;
    if (passed == true)
      std::cerr << "Status: PASSED" << std::endl;
    else
      std::cerr << "Status: FAILED" << std::endl;

    std::cerr << "Time: " << std::to_string(exec_time).c_str()
      << "\n" << std::endl;
  });
}

struct rand_gen
{
  boost::random::mt19937 m_rng;
  typedef boost::random::uniform_int_distribution<size_t> rng_dist_t;

  rand_gen(size_t seed=std::time(0))
    : m_rng(seed)
  { }

  size_t rand(size_t min, size_t max)
  {
    if (max == std::numeric_limits<size_t>::max())
      return rng_dist_t(min, max-1)(m_rng);
    else
      return rng_dist_t(min, max)(m_rng);
  }
};

struct check_stlproxy
{
private:
  stapl::stream<std::ifstream> m_zin;

public:
  check_stlproxy(stapl::stream<std::ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef size_t result_type;
  template <typename InCont>
  result_type operator()(InCont view)
  {
    std::size_t num;
    for (auto iter = view.begin(); iter != view.end(); ++iter) {
      m_zin >> num;
      (*iter) = num;
    }
    auto container_size = view.size();

    std::size_t five = 5;

    view.push_back(five);
    if (container_size == view.size())
      std::cerr << "Push Back Failed" << std::endl;

    auto container_sum = std::accumulate(view.begin(), view.end(), 0,
     stapl::plus<size_t>());

    view.insert(view.begin()+2, five);
    if ((container_sum+5) != std::accumulate(view.begin(), view.end(), 0,
      stapl::plus<size_t>()))
      std::cerr << "Insert(iterator_position, value) failed." << std::endl;


    view.insert(view.begin()+2, 1, five);
    if ((container_sum+10) != std::accumulate(view.begin(), view.end(), 0,
      stapl::plus<size_t>()))
      std::cerr << "Insert(iterator_position, value) failed." << std::endl;

    view.insert(view.begin()+2, view.begin(), view.begin()+2);

    auto rb = view.rbegin();
    auto re = view.rend();
    auto cb = view.cbegin();
    auto ce = view.cend();
    auto crb = view.crbegin();
    auto cre = view.crend();

    if (view.max_size() <= 0)
      std::cerr << "std::vector.max_size() FAILED" << std::endl;

    if (view.empty())
      std::cerr << "std::vector.empty() FAILED" << std::endl;

    view.reserve(23);

    auto contsize = view.size();
    view.erase(view.begin());

    if (contsize-1 != view.size())
      std::cerr << "std::vector.erase(iterator_position) FAILED" << std::endl;

    view.erase(view.begin(), view.begin()+3);
    if (contsize-4 != view.size())
      std::cerr << "std::vector.erase(iterator_first, iterator_last) FAILED"
        << std::endl;

    std::vector<std::size_t> first;

    first.assign(7,100);
    auto firstsum = std::accumulate(first.rbegin(), first.rend(), 0,
      stapl::plus<size_t>());

    if (firstsum != 700)
      std::cerr << "std::vector.assign FAILED " << std::endl;

   // TODO:  Emplace methods need to be checked with types that are 
   // moveable but not copyable.
    auto it = first.emplace(first.begin(), 100);

    auto firstsumtwo = std::accumulate(first.crbegin(), first.crend(), 0,
      stapl::plus<size_t>());

    if (firstsumtwo != 800)
      std::cerr << "std::vector.emplace FAILED " << std::endl;

    first.emplace_back(100);

    auto firstsumthree = std::accumulate(first.crbegin(), first.crend(), 0,
      stapl::plus<size_t>());

    if (firstsumthree != 900)
      std::cerr << "std::vector.emplace_back FAILED " << std::endl;

    auto firstsize = first.size();

    first.resize(firstsize+1);

    if (first.size() != firstsize+1)
      std::cerr << "std::vector.resize() FAILED" << std::endl;

    return std::accumulate(view.begin(), view.end(), 0, xor_un_wf());
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};

struct l1_seq_l0_seq_cksum_wf
{
  typedef size_t result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return std::accumulate(elem.begin(), elem.end(), 0, xor_un_wf());

  }
};

void p_vector(size_t n, int num_runs, stapl::stream<std::ifstream>& zin)
{
  rand_gen gen;
  n = gen.rand(1, n) * 10;

  stapl::tuple<bool, double> fin_res(true, 0.0);
  for (int i = 0; i < num_runs; i++)
  {
    typedef stapl::vector<std_cont> p_vec_vec_tp;
    typedef stapl::vector_view<p_vec_vec_tp> p_vec_vec_vw_tp;

    std_cont vec(gen.rand(1, n));
    p_vec_vec_tp p_vec_vec(gen.rand(1, n), vec);
    p_vec_vec_vw_tp p_vec_vec_vw(p_vec_vec);

    size_t xor_vals = stapl::map_reduce(check_stlproxy(zin), xor_un_wf(),
      p_vec_vec_vw);

    stapl::counter<stapl::default_timer>ctr;
    ctr.start();
    size_t cksum = stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      p_vec_vec_vw);
    stapl::get<1>(fin_res) += ctr.stop();
    stapl::get<0>(fin_res) = (xor_vals == cksum) && stapl::get<0>(fin_res);
    stapl::rmi_fence();
  }
  print_results("p_vector<std::vector>", fin_res, num_runs);
}

stapl::exit_code stapl_main(int argc, char** argv)
{
  size_t data_size = 1;
  int iterations = 2;

  std::string inputfname = "data/tiny_bits.zin";
  stapl::stream<std::ifstream> zin;
  zin.open(inputfname.c_str());

  p_vector(data_size, iterations, zin);

  return EXIT_SUCCESS;
}
