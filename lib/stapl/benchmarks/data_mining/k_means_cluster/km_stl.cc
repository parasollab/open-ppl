/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// This define to disable bounds checking in boost::multi_array
#define BOOST_DISABLE_ASSERTS

#include <cfloat>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <boost/multi_array.hpp>
#include <boost/array.hpp>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/tuple/tuple.hpp>

#include "confint.hpp"

using namespace std;

#define TIMER 1

////////////////////////////////////////////////////////////////////////////////
inline double euclid_dist_sq(double x, double y)
{
  const double temp = x - y;
  return temp * temp;
}


////////////////////////////////////////////////////////////////////////////////
// read in the data points
// select initial cluster centroids
////////////////////////////////////////////////////////////////////////////////
template<typename Stream, typename Points, typename Centroids>
void kmeans_input(Stream& in_str, Points& points, Centroids& centroids)
{
  typedef decltype(*points.begin()) point_ref;

  std::for_each(points.begin(), points.end(), [&](point_ref&& point) {
    in_str.read((char*) &point[0], sizeof(double) * point.size());

    if (in_str.fail())
    {
      cerr << "Incomplete file read" << endl;
      exit(1);
    }
  });

  std::copy(points.begin(), points.begin() + centroids.size(),
            centroids.begin());
}


////////////////////////////////////////////////////////////////////////////////
// display results to standard output
////////////////////////////////////////////////////////////////////////////////
template<typename Centroids>
void kmeans_output(int k_clusters, int point_count, int dimensions,
                   Centroids& centroids)
{
  cout << "k= " << k_clusters << ", n= " << point_count << ", ";
  cout << "d= " << dimensions << endl;
  cout << "CENTROIDS" << endl;

  typedef decltype(*centroids.begin()) centroid_ref;

  std::for_each(centroids.begin(), centroids.end(),[&](centroid_ref&& centroid){
    std::for_each(centroid.begin(), centroid.end(),
                  [&](double const& dimension) { cout << dimension << " "; });
    cout << endl; });
}


////////////////////////////////////////////////////////////////////////////////
// k-means computation
////////////////////////////////////////////////////////////////////////////////
template<typename Points, typename Centroids, typename Counts>
void kmeans_seq(Counts& old_assign_counts, Centroids& old_centroids,
                Counts& new_assign_counts, Centroids& new_centroids,
                Points& points, double& mean_sq_err, int& iterations) {

  std::vector<double> distance(old_centroids.size());
  double              old_mean_sq_err;
  double              new_mean_sq_err;
  std::plus<double>   plus_wf;

  typedef decltype(*(old_centroids.begin())) point_ref;

  do {
    old_mean_sq_err = mean_sq_err;
    new_mean_sq_err = 0.0;

    // initialize for another iteration
    std::fill(new_assign_counts.begin(), new_assign_counts.end(), 0);

    for_each(new_centroids.begin(), new_centroids.end(),
             [](point_ref&& centroid) {
               std::fill(centroid.begin(), centroid.end(), 0.0); });

    // for all points
    for_each(points.begin(), points.end(), [&](point_ref&& point) {
      // compute squared Euclidean distance from all centroids to this point
      std::transform(
        old_centroids.begin(), old_centroids.end(), distance.begin(),
        [&](point_ref&& centroid) {
          return std::inner_product(point.begin(),point.end(),centroid.begin(),
                                    0, plus_wf, euclid_dist_sq); });

      // find the nearest centroid to this point
      auto iter         = std::min_element(distance.begin(), distance.end());
      const int nearest = std::distance(distance.begin(), iter);

      // update the new owner centroid to include this point
      std::transform(new_centroids[nearest].begin(),
                     new_centroids[nearest].end(), point.begin(),
                     new_centroids[nearest].begin(), plus_wf);

      ++new_assign_counts[nearest];

      new_mean_sq_err += std::inner_product(point.begin(), point.end(),
                                            old_centroids[nearest].begin(), 0,
                                            plus_wf, euclid_dist_sq);
    });

    // re-estimate the centroids as the average of the assigned points
    std::transform(new_assign_counts.begin(), new_assign_counts.end(),
                   old_assign_counts.begin(),
                   [](int& new_count) { return std::max(new_count, 1); });

    auto beg_iter = boost::make_zip_iterator(boost::make_tuple(
      old_centroids.begin(), new_centroids.begin(), old_assign_counts.begin()));

    auto end_iter = boost::make_zip_iterator(boost::make_tuple(
      old_centroids.end(), new_centroids.end(), old_assign_counts.end()));

    typedef decltype(*beg_iter) tuple_ref;

    std::for_each(beg_iter, end_iter, [](tuple_ref&& t) {
      std::transform(boost::get<1>(t).begin(), boost::get<1>(t).end(),
                     boost::get<0>(t).begin(), [&](double const& dimension) {
                       return dimension / boost::get<2>(t); }); });

    ++iterations;

    mean_sq_err = new_mean_sq_err;
  } while (mean_sq_err < old_mean_sq_err);
}


////////////////////////////////////////////////////////////////////////////////
// driver
////////////////////////////////////////////////////////////////////////////////
void kmeans(int k_clusters, int point_count, int dimensions, ifstream& in_str)
{
#ifdef TIMER
  counter_t timer;
  timer.reset();
  timer.start();
#endif

  typedef boost::multi_array<double, 2> points_ct_t;
  typedef std::vector<int>              counts_ct_t;

  const auto points_extents    = boost::extents[point_count][dimensions];
  const auto centroids_extents = boost::extents[ k_clusters][dimensions];

  points_ct_t points(points_extents);

#ifdef TIMER
  points_ct_t initial_centroids(centroids_extents);
#endif

  points_ct_t old_centroids(centroids_extents);
  points_ct_t new_centroids(centroids_extents);

  counts_ct_t old_assign_counts(k_clusters);
  counts_ct_t new_assign_counts(k_clusters);

  double mean_sq_err = DBL_MAX;
  int iterations     = 0;

  kmeans_input(in_str, points, old_centroids);

  initial_centroids = old_centroids;

#ifdef TIMER
  double io_time = timer.stop();

  confidence_interval_controller iter_control(32, 100, 0.05);

  while (iter_control.iterate())
  {
    old_centroids = initial_centroids;

    timer.reset();
    timer.start();
#endif

  kmeans_seq(old_assign_counts, old_centroids, new_assign_counts, new_centroids,
             points, mean_sq_err, iterations);

#ifdef TIMER
    iter_control.push_back(timer.stop());
  }
#endif

  kmeans_output(k_clusters, point_count, dimensions, old_centroids);

#ifdef TIMER
 iter_control.report("km_seq");
 cout << "io= " << io_time << ",";
 cout << "mse= " << mean_sq_err << ", iter= " << iterations << endl;
#endif
}


////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if ( argc < 2 ) {
    cerr << "Cluster count must be specified on command line" << endl;
    exit(1);
  }

  int k_clusters = atoi( argv[1] );

  if ( k_clusters < 2 ) {
    cerr << "Cluster count must be greater than 1" << endl;
    exit(1);
  }

  if ( argc < 3 ) {
    cerr << "Point count must be specified on command line" << endl;
    exit(1);
  }

  int point_count = atoi( argv[2] );

  if ( point_count < 1 ) {
    cerr << "Point count must be greater than 0" << endl;
    exit(1);
  }

  if ( argc < 4 ) {
    cerr << "Dimension count must be specified on command line" << endl;
    exit(1);
  }

  int dimensions = atoi( argv[3] );

  if ( point_count < 2 ) {
    cerr << "Dimensions must be greater than 1" << endl;
    exit(1);
  }

  if ( argc < 5 ) {
    cerr << "Input file must be specified on command line" << endl;
    exit(1);
  }

  ifstream in_str;
  in_str.open(argv[4], ios::binary|ios::in );

  if ( !in_str.is_open() ) {
    cerr << "Input file must be specified on command line" << endl;
    exit(1);
  }

  kmeans(k_clusters, point_count, dimensions, in_str);

  in_str.close();
}
