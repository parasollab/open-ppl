#include <boost/lexical_cast.hpp>
#include <stapl/set.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include "../../confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;
typedef stapl::distribution_spec<> distribution_spec;

struct redist_times
{
  std::vector<double> ctor_time;
  std::vector<double> gen_time;
  std::vector<double> redist_time;
  std::vector<double> acc_time;
  bool                correct;

  redist_times(int samples)
    : correct(true)
  {
    ctor_time.reserve(samples);
    gen_time.reserve(samples);
    redist_time.reserve(samples);
    acc_time.reserve(samples);
  }
};


struct get_second
{
  typedef long int result_type;

  template <typename Element>
  result_type operator()(Element e) const
  { return e.second; }
};


void compute(distribution_spec& orig, distribution_spec& target,
             redist_times& times)
{
  typedef distribution_spec partitioning_view_type;
  typedef stapl::set<long int, stapl::less<long int>,
            stapl::view_based_partition<partitioning_view_type>,
            stapl::view_based_mapper<partitioning_view_type> > container_t;

  stapl::location_type lid = stapl::get_location_id();
  stapl::location_type nlocs = stapl::get_num_locations();

  counter_t ctor_timer, redist_timer, gen_timer, acc_timer;
  ctor_timer.reset();
  redist_timer.reset();
  gen_timer.reset();
  acc_timer.reset();

  ctor_timer.start();
  container_t c(orig);
  times.ctor_time.push_back(ctor_timer.stop());

  stapl::set_view<container_t> cv(c);

  gen_timer.start();
  long int n = orig.size();
  long int num_big_blocks = n % nlocs;
  long int block = n / nlocs + (lid < num_big_blocks ? 1 : 0);
  long int start = block * lid + (lid >= num_big_blocks ? num_big_blocks : 0);
  long int end   = start + block-1;

  for (long int i = start; i < end+1; ++i)
    c.insert(i);
  // Wait on all locations to finish populating the container.
  stapl::rmi_fence();

  times.gen_time.push_back(gen_timer.stop());

  redist_timer.start();
  c.redistribute(target);
  times.redist_time.push_back(redist_timer.stop());

  stapl::set_view<container_t> cv2(c);

  acc_timer.start();
  long int zero = 0;
  long int res = stapl::accumulate(cv2, zero);
  times.acc_time.push_back(acc_timer.stop());
  times.correct = times.correct && res == (n-1)*n/2;
  stapl::rmi_fence();
}

stapl::exit_code stapl_main(int argc, char** argv)
{
  long int n;
  long int blk_sz;
  int samples;
  if (argc > 3)
  {
    n = boost::lexical_cast<long int>(argv[1]);
    blk_sz = boost::lexical_cast<long int>(argv[2]);
    samples = boost::lexical_cast<long int>(argv[3]);
  }
  else
  {
    n = 1025;
    blk_sz = 16;
    samples = 1;
  }

  redist_times bal_blk_times(samples);
  redist_times bal_blk_cyc_times(samples);
  redist_times bal_cyc_times(samples);

  redist_times blk_bal_times(samples);
  redist_times blk_blk_cyc_times(samples);
  redist_times blk_cyc_times(samples);

  redist_times blk_cyc_bal_times(samples);
  redist_times blk_cyc_blk_times(samples);
  redist_times blk_cyc_cyc_times(samples);

  distribution_spec bal_spec = stapl::balance(n);
  distribution_spec blk_spec = stapl::block(n,blk_sz);
  distribution_spec blk_cyc_spec = stapl::block_cyclic(n,blk_sz);
  distribution_spec cyc_spec = stapl::cyclic(n);

  for (int iter = 0; iter != samples; ++iter)
  {
    compute(bal_spec,     blk_spec,     bal_blk_times);
    compute(bal_spec,     blk_cyc_spec, bal_blk_cyc_times);
    compute(bal_spec,     cyc_spec,     bal_cyc_times);

    compute(blk_spec,     blk_cyc_spec, blk_blk_cyc_times);
    compute(blk_spec,     bal_spec,     blk_bal_times);
    compute(blk_spec,     cyc_spec,     blk_cyc_times);

    compute(blk_cyc_spec, bal_spec,     blk_cyc_bal_times);
    compute(blk_cyc_spec, blk_spec,     blk_cyc_blk_times);
    compute(blk_cyc_spec, cyc_spec,     blk_cyc_cyc_times);
  }

  // balanced to others
  {
  report("bal_blk_ctr", bal_blk_times.ctor_time, bal_blk_times.correct);
  report("bal_blk_gen", bal_blk_times.gen_time, bal_blk_times.correct);
  report("bal_blk_rdt", bal_blk_times.redist_time, bal_blk_times.correct);
  report("bal_blk_acc", bal_blk_times.acc_time, bal_blk_times.correct);

  report("bal_blk_cyc_ctr", bal_blk_cyc_times.ctor_time,
         bal_blk_cyc_times.correct);
  report("bal_blk_cyc_gen", bal_blk_cyc_times.gen_time,
         bal_blk_cyc_times.correct);
  report("bal_blk_cyc_rdt", bal_blk_cyc_times.redist_time,
         bal_blk_cyc_times.correct);
  report("bal_blk_cyc_acc", bal_blk_cyc_times.acc_time,
         bal_blk_cyc_times.correct);

  report("bal_cyc_ctr", bal_cyc_times.ctor_time, bal_cyc_times.correct);
  report("bal_cyc_gen", bal_cyc_times.gen_time, bal_cyc_times.correct);
  report("bal_cyc_rdt", bal_cyc_times.redist_time, bal_cyc_times.correct);
  report("bal_cyc_acc", bal_cyc_times.acc_time, bal_cyc_times.correct);
  }

  // block to others
  {
  report("blk_bal_ctr", blk_bal_times.ctor_time, blk_bal_times.correct);
  report("blk_bal_gen", blk_bal_times.gen_time, blk_bal_times.correct);
  report("blk_bal_rdt", blk_bal_times.redist_time, blk_bal_times.correct);
  report("blk_bal_acc", blk_bal_times.acc_time, blk_bal_times.correct);

  report("blk_blk_cyc_ctr", blk_blk_cyc_times.ctor_time,
         blk_blk_cyc_times.correct);
  report("blk_blk_cyc_gen", blk_blk_cyc_times.gen_time,
         blk_blk_cyc_times.correct);
  report("blk_blk_cyc_rdt", blk_blk_cyc_times.redist_time,
         blk_blk_cyc_times.correct);
  report("blk_blk_cyc_acc", blk_blk_cyc_times.acc_time,
         blk_blk_cyc_times.correct);

  report("blk_cyc_ctr", blk_cyc_times.ctor_time,
         blk_cyc_times.correct);
  report("blk_cyc_gen", blk_cyc_times.gen_time,
         blk_cyc_times.correct);
  report("blk_cyc_rdt", blk_cyc_times.redist_time,
         blk_cyc_times.correct);
  report("blk_cyc_acc", blk_cyc_times.acc_time,
         blk_cyc_times.correct);
  }

  // block-cyclic to others
  {
  report("blk_cyc_bal_ctr", blk_cyc_bal_times.ctor_time,
         blk_cyc_bal_times.correct);
  report("blk_cyc_bal_gen", blk_cyc_bal_times.gen_time,
         blk_cyc_bal_times.correct);
  report("blk_cyc_bal_rdt", blk_cyc_bal_times.redist_time,
         blk_cyc_bal_times.correct);
  report("blk_cyc_bal_acc", blk_cyc_bal_times.acc_time,
         blk_cyc_bal_times.correct);

  report("blk_cyc_blk_ctr", blk_cyc_blk_times.ctor_time,
         blk_cyc_blk_times.correct);
  report("blk_cyc_blk_gen", blk_cyc_blk_times.gen_time,
         blk_cyc_blk_times.correct);
  report("blk_cyc_blk_rdt", blk_cyc_blk_times.redist_time,
         blk_cyc_blk_times.correct);
  report("blk_cyc_blk_acc", blk_cyc_blk_times.acc_time,
         blk_cyc_blk_times.correct);

  report("blk_cyc_cyc_ctr", blk_cyc_cyc_times.ctor_time,
         blk_cyc_cyc_times.correct);
  report("blk_cyc_cyc_gen", blk_cyc_cyc_times.gen_time,
         blk_cyc_cyc_times.correct);
  report("blk_cyc_cyc_rdt", blk_cyc_cyc_times.redist_time,
         blk_cyc_cyc_times.correct);
  report("blk_cyc_cyc_acc", blk_cyc_cyc_times.acc_time,
         blk_cyc_cyc_times.correct);
  }
  return EXIT_SUCCESS;
}
