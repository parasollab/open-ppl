#include <boost/lexical_cast.hpp>
#include <stapl/graph.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/numeric.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/system_view.hpp>
#include "../../confint.hpp"

typedef stapl::counter<stapl::default_timer> counter_t;
typedef stapl::distribution_spec<> distribution_spec;

struct assign_prop
{
  typedef void result_type;

  template <typename Vertex, typename Index>
  result_type operator()(Vertex v, Index i)
  {
    v.property() = i;
  }
};

struct get_prop
{
  typedef long int result_type;

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    return v.property();
  }
};

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

void compute(distribution_spec& orig, distribution_spec& target,
             redist_times& times)
{
  typedef distribution_spec partitioning_view_type;
  typedef stapl::graph<stapl::DIRECTED, stapl::MULTIEDGES, long int, long int,
            stapl::view_based_partition<partitioning_view_type>,
            stapl::view_based_mapper<partitioning_view_type> > container_t;

  counter_t ctor_timer, redist_timer, gen_timer, acc_timer;
  ctor_timer.reset();
  redist_timer.reset();
  gen_timer.reset();
  acc_timer.reset();

  ctor_timer.start();
  container_t c(orig);
  times.ctor_time.push_back(ctor_timer.stop());

  stapl::graph_view<container_t> cv(c);

  gen_timer.start();
  stapl::map_func(assign_prop(), cv,
                  stapl::counting_view<long int>(orig.domain().size()));
  times.gen_time.push_back(gen_timer.stop());

  redist_timer.start();
  c.redistribute(target);
  times.redist_time.push_back(redist_timer.stop());

  stapl::graph_view<container_t> cv2(c);

  acc_timer.start();
  long int res = stapl::map_reduce(get_prop(), stapl::plus<long int>(), cv2);
  times.acc_time.push_back(acc_timer.stop());
  long int n = c.size();
  times.correct = times.correct && res == (n-1)*n/2;
  stapl::rmi_fence();
}

void compute_dynamic(distribution_spec& orig, distribution_spec& target,
                     redist_times& times)
{
  typedef distribution_spec partitioning_view_type;
  typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES,
            long int, long int,
            stapl::view_based_partition<partitioning_view_type>,
            stapl::view_based_mapper<partitioning_view_type> > container_t;

  counter_t ctor_timer, redist_timer, gen_timer, acc_timer;
  ctor_timer.reset();
  redist_timer.reset();
  gen_timer.reset();
  acc_timer.reset();

  ctor_timer.start();
  container_t c(orig);
  times.ctor_time.push_back(ctor_timer.stop());

  stapl::graph_view<container_t> cv(c);

  gen_timer.start();
  stapl::map_func(assign_prop(), cv,
                  stapl::counting_view<long int>(orig.domain().size()));
  times.gen_time.push_back(gen_timer.stop());

  redist_timer.start();
#if 0
  // re-enable when redistribution of dynamic_graph in shared memory no longer
  // results in a hang due to base container iterator issues.
  c.redistribute(target);
#endif
  times.redist_time.push_back(redist_timer.stop());

  stapl::graph_view<container_t> cv2(c);

  acc_timer.start();
  long int res = stapl::map_reduce(get_prop(), stapl::plus<long int>(), cv2);
  times.acc_time.push_back(acc_timer.stop());
  long int n = c.size();
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

  redist_times dyn_bal_blk_times(samples);
  redist_times dyn_bal_blk_cyc_times(samples);
  redist_times dyn_bal_cyc_times(samples);

  redist_times dyn_blk_bal_times(samples);
  redist_times dyn_blk_blk_cyc_times(samples);
  redist_times dyn_blk_cyc_times(samples);

  redist_times dyn_blk_cyc_bal_times(samples);
  redist_times dyn_blk_cyc_blk_times(samples);
  redist_times dyn_blk_cyc_cyc_times(samples);

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

    compute_dynamic(bal_spec,     blk_spec,     dyn_bal_blk_times);
    compute_dynamic(bal_spec,     blk_cyc_spec, dyn_bal_blk_cyc_times);
    compute_dynamic(bal_spec,     cyc_spec,     dyn_bal_cyc_times);

    compute_dynamic(blk_spec,     blk_cyc_spec, dyn_blk_blk_cyc_times);
    compute_dynamic(blk_spec,     bal_spec,     dyn_blk_bal_times);
    compute_dynamic(blk_spec,     cyc_spec,     dyn_blk_cyc_times);

    compute_dynamic(blk_cyc_spec, bal_spec,     dyn_blk_cyc_bal_times);
    compute_dynamic(blk_cyc_spec, blk_spec,     dyn_blk_cyc_blk_times);
    compute_dynamic(blk_cyc_spec, cyc_spec,     dyn_blk_cyc_cyc_times);
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

  report("blk_cyc_ctr", blk_cyc_times.ctor_time, blk_cyc_times.correct);
  report("blk_cyc_gen", blk_cyc_times.gen_time, blk_cyc_times.correct);
  report("blk_cyc_rdt", blk_cyc_times.redist_time, blk_cyc_times.correct);
  report("blk_cyc_acc", blk_cyc_times.acc_time, blk_cyc_times.correct);
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


  // dynamic balanced to others
  {
  report("dyn_bal_blk_ctr", dyn_bal_blk_times.ctor_time,
         dyn_bal_blk_times.correct);
  report("dyn_bal_blk_gen", dyn_bal_blk_times.gen_time,
         dyn_bal_blk_times.correct);
  report("dyn_bal_blk_rdt", dyn_bal_blk_times.redist_time,
         dyn_bal_blk_times.correct);
  report("dyn_bal_blk_acc", dyn_bal_blk_times.acc_time,
         dyn_bal_blk_times.correct);

  report("dyn_bal_blk_cyc_ctr", dyn_bal_blk_cyc_times.ctor_time,
         dyn_bal_blk_cyc_times.correct);
  report("dyn_bal_blk_cyc_gen", dyn_bal_blk_cyc_times.gen_time,
         dyn_bal_blk_cyc_times.correct);
  report("dyn_bal_blk_cyc_rdt", dyn_bal_blk_cyc_times.redist_time,
         dyn_bal_blk_cyc_times.correct);
  report("dyn_bal_blk_cyc_acc", dyn_bal_blk_cyc_times.acc_time,
         dyn_bal_blk_cyc_times.correct);

  report("dyn_bal_cyc_ctr", dyn_bal_cyc_times.ctor_time,
         dyn_bal_cyc_times.correct);
  report("dyn_bal_cyc_gen", dyn_bal_cyc_times.gen_time,
         dyn_bal_cyc_times.correct);
  report("dyn_bal_cyc_rdt", dyn_bal_cyc_times.redist_time,
         dyn_bal_cyc_times.correct);
  report("dyn_bal_cyc_acc", dyn_bal_cyc_times.acc_time,
         dyn_bal_cyc_times.correct);
  }


  // dynamic block to others
  {
  report("dyn_blk_bal_ctr", dyn_blk_bal_times.ctor_time,
         dyn_blk_bal_times.correct);
  report("dyn_blk_bal_gen", dyn_blk_bal_times.gen_time,
         dyn_blk_bal_times.correct);
  report("dyn_blk_bal_rdt", dyn_blk_bal_times.redist_time,
         dyn_blk_bal_times.correct);
  report("dyn_blk_bal_acc", dyn_blk_bal_times.acc_time,
         dyn_blk_bal_times.correct);

  report("dyn_blk_blk_cyc_ctr", dyn_blk_blk_cyc_times.ctor_time,
         dyn_blk_blk_cyc_times.correct);
  report("dyn_blk_blk_cyc_gen", dyn_blk_blk_cyc_times.gen_time,
         dyn_blk_blk_cyc_times.correct);
  report("dyn_blk_blk_cyc_rdt", dyn_blk_blk_cyc_times.redist_time,
         dyn_blk_blk_cyc_times.correct);
  report("dyn_blk_blk_cyc_acc", dyn_blk_blk_cyc_times.acc_time,
         dyn_blk_blk_cyc_times.correct);

  report("dyn_blk_cyc_ctr", dyn_blk_cyc_times.ctor_time,
         dyn_blk_cyc_times.correct);
  report("dyn_blk_cyc_gen", dyn_blk_cyc_times.gen_time,
         dyn_blk_cyc_times.correct);
  report("dyn_blk_cyc_rdt", dyn_blk_cyc_times.redist_time,
         dyn_blk_cyc_times.correct);
  report("dyn_blk_cyc_acc", dyn_blk_cyc_times.acc_time,
         dyn_blk_cyc_times.correct);
  }


  // dynamic block-cyclic to others
  {
  report("dyn_blk_cyc_bal_ctr", dyn_blk_cyc_bal_times.ctor_time,
         dyn_blk_cyc_bal_times.correct);
  report("dyn_blk_cyc_bal_gen", dyn_blk_cyc_bal_times.gen_time,
         dyn_blk_cyc_bal_times.correct);
  report("dyn_blk_cyc_bal_rdt", dyn_blk_cyc_bal_times.redist_time,
         dyn_blk_cyc_bal_times.correct);
  report("dyn_blk_cyc_bal_acc", dyn_blk_cyc_bal_times.acc_time,
         dyn_blk_cyc_bal_times.correct);

  report("dyn_blk_cyc_blk_ctr", dyn_blk_cyc_blk_times.ctor_time,
         dyn_blk_cyc_blk_times.correct);
  report("dyn_blk_cyc_blk_gen", dyn_blk_cyc_blk_times.gen_time,
         dyn_blk_cyc_blk_times.correct);
  report("dyn_blk_cyc_blk_rdt", dyn_blk_cyc_blk_times.redist_time,
         dyn_blk_cyc_blk_times.correct);
  report("dyn_blk_cyc_blk_acc", dyn_blk_cyc_blk_times.acc_time,
         dyn_blk_cyc_blk_times.correct);

  report("dyn_blk_cyc_cyc_ctr", dyn_blk_cyc_cyc_times.ctor_time,
         dyn_blk_cyc_cyc_times.correct);
  report("dyn_blk_cyc_cyc_gen", dyn_blk_cyc_cyc_times.gen_time,
         dyn_blk_cyc_cyc_times.correct);
  report("dyn_blk_cyc_cyc_rdt", dyn_blk_cyc_cyc_times.redist_time,
         dyn_blk_cyc_cyc_times.correct);
  report("dyn_blk_cyc_cyc_acc", dyn_blk_cyc_cyc_times.acc_time,
         dyn_blk_cyc_cyc_times.correct);
  }

  return EXIT_SUCCESS;
}
