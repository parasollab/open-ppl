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
  {
    num = random_distribution_t(0.0, 1.0)(m_rng);
  }

  void define_type(stapl::typer& t)
  {
    stapl::abort("Attempt to serialize rand_wf. std::mt19937 not serializable");
  }
};

struct grind_wf
{
  template<typename NumRef>
  void operator()(NumRef&& n)
  {
    for (int k=0; k < 1000; ++k)
      n = n + (k*k/100);
  }
};

struct over_point_five
{
  template<typename ElemRef>
  bool operator()(ElemRef&& e) const
  { return e > 0.5; }
};

stapl::exit_code stapl_main(int argc, char*argv[])
{
  using container = stapl::static_array<double>;
  using view      = stapl::array_view<container>;
  using domain    = stapl::indexed_domain<size_t>;

  int size  = atoi(argv[1]);

  container nums(size);
  view nums_view(nums);

  container numsOut(size);
  view numsOut_view(nums);

  // Init
  stapl::fill(nums_view, 4);
  stapl::fill(numsOut_view, 2);
  stapl::map_func(rand_wf(), nums_view);

#if DEBUG
  stapl::do_once([]{ std::cout << "before:"; });
  stapl::serial(print_wf(), nums_view);
  stapl::do_once([]{ std::cout << "\n"; });
#endif

  // begin timing, default timer measures time and reports it in seconds
  stapl::counter<stapl::default_timer> timer;
  timer.reset();
  timer.start();

  // Apply sort
  stapl::sort(nums_view);

  // Apply unique_copy
  stapl::unique_copy(nums_view, numsOut_view);

  // Apply for_each
  stapl::for_each(numsOut_view, grind_wf());

  // Finally, we will apply a simple "find_if" routine
  auto element = stapl::find_if(numsOut_view, over_point_five());

  double time = timer.stop();

  stapl::do_once(
    [time, size, element]{
      std::cout << "The first value is " << element << '\n';
      std::cout << "mix " << size
                << " Execution time: " << time << "\n";
  });

#if DEBUG
  stapl::do_once([]{ std::cout << "after:"; });
  stapl::serial(print_wf(), numsOut_view);
  stapl::do_once([]{ std::cout << "\n"; });
#endif

  return EXIT_SUCCESS;
}
