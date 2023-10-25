#include <stapl/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/skeletons/serial.hpp>
#include <iostream>

struct print_wf
{
  template<typename NumRef>
  void operator()(NumRef&& n)
  { std::cout << ' ' << n; }
};


struct init_wf
{
private:
  size_t m_num_duplicates;

public:
  init_wf(size_t num_duplicates)
    : m_num_duplicates(num_duplicates)
  { }

  template<typename NumRef, typename NumOutRef, typename Index>
  void operator()(NumRef&& num, NumOutRef&& num_out, Index&& i)
  {
    if (m_num_duplicates == 0 || i > m_num_duplicates-1)
      num = i;

    num_out = 2;
  }

  void define_type(stapl::typer& t)
  { t.member(m_num_duplicates); }
};


stapl::exit_code stapl_main(int argc, char*argv[])
{
  using container = stapl::static_array<double>;
  using view      = stapl::array_view<container>;

  int size = atoi(argv[1]);
  int num_duplicates = argc == 3 ? atoi(argv[2]) : 27;

  container nums(size);
  view nums_view(nums);

  container numsOut(size);
  view numsOut_view(numsOut);

  // Init
  stapl::fill(nums_view, 24);
  stapl::map_func(init_wf(num_duplicates), nums_view, numsOut_view,
                  stapl::counting_view<int>(size));

#if DEBUG
  stapl::do_once([]{ std::cout << "before:"; });

  stapl::serial(print_wf(), nums_view);
#endif

  // begin timing, default timer measures time and reports it in seconds
  stapl::counter<stapl::default_timer> timer;
  timer.reset();
  timer.start();

  // Apply unique_copy
  stapl::unique_copy(nums_view, numsOut_view);

  double time = timer.stop();
  stapl::do_once(
    [time,size]{
      std::cout << "unique_copy " << size
                << " Execution time: " << time << "\n";
  });

#if DEBUG
  stapl::do_once([]{ std::cout << "after:"; });

  stapl::serial(print_wf(), numsOut_view);
#endif

  if (size <= 100000)
    stapl::do_once([size, &nums_view, &numsOut_view, num_duplicates]()
    {
      std::vector<int> v;
      std::vector<int> q;

      v.resize(size);
      q.resize(size);

      for (int i=0; i<size; ++i)
        v[i] = nums_view[i];

      std::unique_copy(v.begin(), v.end(), q.begin());

      bool pass = true;

      for (int i=0; i< (size - num_duplicates - 1) ; ++i)
      {
        if (q[i] != numsOut_view[i])
        {
          std::cout << "FAIL on " << i << " ===> "
                    << q[i] << " // " << numsOut_view[i] << "\n";
          pass = false;
        }
      }

      if (pass)
        std::cout << "PASS\n";
      else
        std::cout << "FAIL\n";
    });

  return EXIT_SUCCESS;
}
