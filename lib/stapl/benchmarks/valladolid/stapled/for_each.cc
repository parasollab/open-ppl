#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/skeletons/serial.hpp>
#include <iostream>

struct print_wf
{
  template<typename NumRef>
  void operator()(NumRef&& n)
  { std::cout << ' ' << n; }
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

stapl::exit_code stapl_main(int argc, char*argv[])
{
  using container = stapl::static_array<double>;
  using view      = stapl::array_view<container>;

  int begin = atoi(argv[1]);
  int end   = atoi(argv[2]);
  int size  = atoi(argv[3]);

  container nums(size);
  view nums_view(nums);

  // Init
  stapl::fill(nums_view, 4);

#if DEBUG
  stapl::do_once([]{ std::cout << "before:"; });

  stapl::serial(print_wf(), nums_view);
#endif

  // begin timing, default timer measures time and reports it in seconds
  stapl::counter<stapl::default_timer> timer;
  timer.reset();
  timer.start();

  // Apply for_each
  view restricted_view(nums, stapl::indexed_domain<size_t>(begin, end-1));
  stapl::for_each(restricted_view, grind_wf());

  double time = timer.stop();
  stapl::do_once([time, size]{
    std::cout << "for_each " << size
              << " Execution time: " << time << "\n";
  });

#if DEBUG
  stapl::do_once([]{ std::cout << "after:"; });

  stapl::serial(print_wf(), nums_view);
#endif

  return EXIT_SUCCESS;
}
