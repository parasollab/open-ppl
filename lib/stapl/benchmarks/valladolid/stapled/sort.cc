#include <stapl/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/skeletons/serial.hpp>
#include <iostream>
#include <random>

struct print_wf
{
  template<typename NumRef>
  void operator()(NumRef&& n)
  { std::cout << ' ' << n; }
};

struct rand_wf
{
private:
  using random_distribution_t = std::uniform_real_distribution<double>;

  std::mt19937 m_rng;

public:
  template<typename NumRef>
  void operator()(NumRef&& num)
  { num = random_distribution_t(0.0, 1.0)(m_rng); }

  void define_type(stapl::typer& t)
  {
    stapl::abort("Attempt to serialize rand_wf. std::mt19937 not serializable");
  }
};

stapl::exit_code stapl_main(int argc, char*argv[])
{
  using container = stapl::static_array<double>;
  using view      = stapl::array_view<container>;

  int size = atoi(argv[1]);

  container nums(size);
  view nums_view(nums);

  // Init
  stapl::fill(nums_view, 0.0);
  stapl::map_func(rand_wf(), nums_view);

#if DEBUG
  stapl::do_once([]{ std::cout << "before:"; });

  stapl::serial(print_wf(), nums_view);
#endif

  // begin timing, default timer measures time and reports it in seconds
  stapl::counter<stapl::default_timer> timer;
  timer.reset();
  timer.start();

  // Apply sort
  stapl::sort(nums_view);

  double time = timer.stop();
  stapl::do_once(
    [time,size]{
      std::cout << "sort " << size
                << " Execution time: " << time << "\n";
  });

#if DEBUG
  stapl::do_once([]{ std::cout << "after:"; });

  stapl::serial(print_wf(), nums_view);
#endif

  return EXIT_SUCCESS;
}
