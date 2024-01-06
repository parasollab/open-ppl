#include <iostream>
#include <vector>

#include "nonstd/stat.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ttest_stat error: ";


void test_average() {
  string when = "when testing average, ";

  vector<int> x = {1, 2, 3, 4};

  assert_msg(approx(average(x), 2.5), er + when + "expected average of "
      "{1, 2, 3, 4} to be 2.5, but got " + to_string(average(x)) + "!");
}


void test_stddev() {
  string when = "when testing stddev, ";

  vector<int> x = {1, 2, 3, 4};

  assert_msg(approx(sample_stddev(x), 1.29099, .0001), er + when +
      "expected sample_stddev of {1, 2, 3, 4} to be 1.29099, but got " +
      to_string(stddev(x)) + "!");
}


void test_entropy() {
  const string when = "when testing entropy, ";

  // Make test instances
  vector<size_t> homogeneous = {2, 0};
  vector<size_t> even = {1, 1};
  vector<size_t> heterogeneous = {2, 1, 3, 4};

  // Exercise tests
  double test_hom = entropy(homogeneous);
  double test_e   = entropy(even);
  double test_het = entropy(heterogeneous);

  // Check correctness
  double tolerance = .000001;
  bool passed_homogeneous   = approx(0., test_hom, tolerance);
  bool passed_even          = approx(1., test_e,   tolerance);
  bool passed_heterogeneous = approx(1.846439345, test_het, tolerance);

  assert_msg(passed_homogeneous, er + when + "failed on homogeneous "
      "set! Expected 0, got " + to_string(test_hom) + "!");
  assert_msg(passed_even, er + when + "failed on even split "
      "set! Expected 1, got " + to_string(test_e) + "!");
  assert_msg(passed_heterogeneous, er + when + "failed on heterogeneous "
      "set! Expected 1.846439345, got " + to_string(test_het) + "!");
}


void test_binomial_distribution() {
  const string when = "when testing binomial_distribution, ";

  // Make test instance
  size_t n        = 4;
  double p        = .5;
  double p_arg    = .375;
  double mean     = 2;
  double variance = 1;
  double std_dev  = 1;
  nonstd::binomial_distribution b(n, p);

  // Call memeber functions on instance
  size_t test_n        = b.num_trials();
  double test_p        = b.probability();
  double test_p_arg    = b.probability(2);
  double test_mean     = b.mean();
  double test_std_dev  = b.std_dev();
  double test_variance = b.variance();

  // Check correctness
  bool passed_n        = n == test_n;
  bool passed_p        = approx(p, test_p);
  bool passed_p_arg    = approx(p_arg, test_p_arg);
  bool passed_mean     = approx(mean, test_mean);
  bool passed_std_dev  = approx(std_dev, test_std_dev);
  bool passed_variance = approx(variance, test_variance);

  assert_msg(passed_n, er + when + "num_trials() failed! expected " +
      to_string(n) + ", got " + to_string(test_n));
  assert_msg(passed_p, er + when + "probability() failed! expected " +
      to_string(p) + ", got " + to_string(test_p));
  assert_msg(passed_p_arg, er + when + "probability(2) failed! expected " +
      to_string(p_arg) + ", got " + to_string(test_p_arg));
  assert_msg(passed_mean, er + when + "mean() failed! expected " +
      to_string(mean) + ", got " + to_string(test_mean));
  assert_msg(passed_std_dev, er + when + "std_dev() failed! expected " +
      to_string(std_dev) + ", got " + to_string(test_std_dev));
  assert_msg(passed_variance, er + when + "variance() failed! expected " +
      to_string(variance) + ", got " + to_string(test_variance));
}


int main() {
  cerr << "\ttesting stat..." << flush;

  test_average();
  test_stddev();
  test_binomial_distribution();
  test_entropy();

  cerr << "passed" << endl;
}
