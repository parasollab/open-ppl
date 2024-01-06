/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <climits>
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/utility/tuple.hpp>

#include "confint.hpp"

using namespace std;

#define TIMER 1

// ---------- declarations ----------

typedef std::vector<double>                                 point_type;
typedef std::vector<point_type>                             centroids_ct_t;
typedef std::vector<int>                                    counts_ct_t;

typedef stapl::result_of::block_cyclic::type                distribution_spec;
typedef stapl::array<point_type,
          stapl::view_based_partition<distribution_spec>,
          stapl::view_based_mapper<distribution_spec>>      points_ct_t;
typedef stapl::array_view<points_ct_t>                      points_vw_t;

typedef stapl::tuple<centroids_ct_t, counts_ct_t, double>   reduction_return_type;

void kmeans(int, int, int, string, int);

template<typename PointsView>
centroids_ct_t kmeans_input(int, int, string, PointsView&);

template<typename PointsView, typename CentroidsView>
tuple<double, int> kmeans_stapl(int, int, int, PointsView&, CentroidsView&);

template<typename CentroidsView>
void kmeans_output(int, int, int, CentroidsView&);

inline double euclid_dist_sq(double x, double y) {
  double temp = x - y;
  return temp * temp;
}

// ---------- stapl_main ----------
// process command line options

stapl::exit_code stapl_main(int argc, char* argv[]) {
   if (argc < 2) {
     cerr << "Cluster count must be specified on command line" << endl;
     exit(1);
   }
   const int k_clusters = atoi( argv[1] );
   if (k_clusters < 2) {
     cerr << "Cluster count must be greater than 1" << endl;
     exit(1);
   }

   if (argc < 3) {
     cerr << "Point count must be specified on command line" << endl;
     exit(1);
   }
   const int point_count = atoi(argv[2]);
   if ( point_count < 1 ) {
     cerr << "Point count must be greater than 0" << endl;
     exit(1);
   }

   if (argc < 4) {
     cerr << "Dimension count must be specified on command line" << endl;
     exit(1);
   }
   const int dimensions = atoi(argv[3]);
   if (point_count < 2) {
     cerr << "Dimensions must be greater than 1" << endl;
     exit(1);
   }

   if (argc < 5) {
     cerr << "Input file must be specified on command line" << endl;
     exit(1);
   }

   if (argc < 6) {
     cerr << "Must specify block size " << endl;
     exit(1);
   }
   int block_size = atoi( argv[5] );

   kmeans(k_clusters, point_count, dimensions, argv[4], block_size);

  return EXIT_SUCCESS;
}

// ---------- kmeans ----------
// allocate storage, read input, perform algorithm, show output

void kmeans(int k_clusters, int point_count, int dimensions, string filename, int block_size) {

#ifdef TIMER
  counter_t timer;
  timer.reset();
  timer.start();
#endif

  distribution_spec blk_cyc_spec = stapl::block_cyclic(point_count, block_size);
  points_ct_t points_ct(blk_cyc_spec, point_type(dimensions, 0.0));
  points_vw_t points_vw(points_ct);

  centroids_ct_t centroids = kmeans_input(k_clusters, dimensions, filename, points_vw);

  double mean_sq_err;
  int    iterations;

#ifdef TIMER
  centroids_ct_t initial_centroids = centroids;

  double io_time = timer.stop();

  bool continue_iterating = true;
  confidence_interval_controller iter_control(32, 100, 0.05);
  while (continue_iterating) {
    centroids = initial_centroids;
    timer.reset();
    timer.start();
#endif

  tie(mean_sq_err, iterations) =
    kmeans_stapl(k_clusters, point_count, dimensions, points_vw, centroids);

#ifdef TIMER
    iter_control.push_back(timer.stop());

    stapl::array<int>                    continue_ct(stapl::get_num_locations());
    stapl::array_view<stapl::array<int>> continue_vw(continue_ct);
    continue_vw[stapl::get_location_id()] = iter_control.iterate() ? 1 : 0;

    int iterate_sum = stapl::accumulate(continue_vw, (int) 0);

    continue_iterating = iterate_sum != 0 ? true : false;
  }
#endif

  kmeans_output(k_clusters, point_count, dimensions, centroids);

#ifdef TIMER
  iter_control.report("km_stapl");
  stapl::do_once([=](void) {
    cout << "io= " << io_time << ",";
    cout << "mse= " << mean_sq_err << ", iter= " << iterations << endl;
  });
#endif
}

// ---------- read_wf ----------
//

struct read_wf {
  typedef void result_type;
  template<typename Indices, typename Points, typename Filename>
  void operator()(Indices indices, Points points, Filename filename) const {

    const int dimensions = (*points.begin()).size();
    const int offset     = *indices.begin() * (dimensions * sizeof(double));

    typename Points::value_type tmp_point(dimensions);

    ifstream in_str(*filename.begin(), ios::binary | ios::in);

    if (!in_str.is_open()) {
      cerr << "Error opening file"<< endl;
      exit(1);
    }
    in_str.seekg(offset);

    for (auto&& point : points) {
      in_str.read((char*) &(*tmp_point.begin()), sizeof(double) * dimensions);

      point = tmp_point;

      if (in_str.fail()) {
        cerr << "Incomplete file read " << endl;
        exit(1);
      }
    }
  }
};

// ----------  kmeans_input ----------
// read data points and select initial cluster centroids

template<typename PointsView>
centroids_ct_t kmeans_input(int k_clusters, int dimensions,
                            string filename, PointsView& points) {

  // read in points parallel
  map_func<stapl::skeletons::tags::with_coarsened_wf>
    (read_wf(), stapl::counting_view<size_t>(points.size()), points,
      stapl::make_repeat_view(filename));

  // read in centroids on one location and broadcat to other locations
  return stapl::do_once<centroids_ct_t>([&](void) {
    centroids_ct_t centroids(k_clusters);

    int size = points.size();
    int step       = size / centroids.size();
    int next       = 0;

    for (auto&& centroid : centroids) {
      centroid = points[next];
      next     = ( next + step ) % size;
    }
    return centroids;
  });
}

// ---------- map_wf ----------
//

struct map_wf {
  typedef reduction_return_type result_type;
  template<typename Points, typename Centroids>
  result_type operator()(Points points, Centroids centroids) const {

    centroids_ct_t   new_centroids(centroids[0].size(),
                                   point_type((*points.begin()).size(), 0.0));

    counts_ct_t      centroid_counts(centroids[0].size(), 0);

    double  mean_sq_err = 0.0;
    int     idx1        = 0;

    for (auto&& p : points) {
      double min_dist  = std::numeric_limits<double>::max();
      int    min_index = -1;
      int index        =  0;

      for (size_t idx2 = 0; idx2 < centroids[idx1].size(); ++idx2) {
        double dist = 0.0;

        for (size_t d=0; d<p.size(); ++d)
          dist += euclid_dist_sq(p[d], centroids[idx1][idx2][d]);

        if (dist < min_dist) {
          min_dist  = dist;
          min_index = index;
        }

        ++index;
      }

      for (size_t d = 0; d < p.size(); ++d)
        new_centroids[min_index][d] += p[d];

      ++centroid_counts[min_index];

      for (size_t d = 0; d < p.size(); ++d) {
        mean_sq_err += euclid_dist_sq(p[d], centroids[0][min_index][d]);
      }
      ++idx1;
    }

    return result_type(new_centroids, centroid_counts, mean_sq_err);
  }
};

// ---------- reduce_wf ----------
//

struct reduce_wf {
  typedef reduction_return_type result_type;
  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 lhs, Reference2 rhs) const {

    result_type lhs_val = lhs;
    result_type rhs_val = rhs;

    centroids_ct_t& lhs_centroids = stapl::get<0>(lhs_val);
    centroids_ct_t& rhs_centroids = stapl::get<0>(rhs_val);
    vector<int>&    lhs_counts    = stapl::get<1>(lhs_val);
    vector<int>&    rhs_counts    = stapl::get<1>(rhs_val);

    const int dimensions = lhs_centroids[0].size();

    for (size_t idx = 0; idx < lhs_centroids.size(); ++idx) {
      // accumulate counts.
      lhs_counts[idx] += rhs_counts[idx];

      // accumulate centroid sums..
      for (int d=0; d<dimensions; ++d)
        lhs_centroids[idx][d] += rhs_centroids[idx][d];
    }

    // Accumulate mean_sq_err
    stapl::get<2>(lhs_val) += stapl::get<2>(rhs_val);

    return lhs_val;
  }
};

// ---------- kmeans_stapl ----------
// perform k-means computation

template<typename PointsView, typename CentroidsView>
tuple<double, int>
kmeans_stapl(int k_clusters, int point_count, int dimensions,
             PointsView& points_vw, CentroidsView& centroids) {

  double      mean_sq_err = DBL_MAX;
  int         iterations  = 0;
  double      old_mean_sq_err;

  counts_ct_t counts;

  do {
    old_mean_sq_err = mean_sq_err;

    // Compute new own centroid for all points.
    stapl::tie(centroids, counts, mean_sq_err) =
      stapl::map_reduce<stapl::skeletons::tags::with_coarsened_wf>
        (map_wf(), reduce_wf(), points_vw, stapl::make_repeat_view(centroids));

    for (auto&& count : counts)
      count = max(count, 1);

    for (size_t i = 0; i < centroids.size(); ++i)
      for (int d=0; d<dimensions; ++d)
        centroids[i][d] /= counts[i];

    ++iterations;
  } while (mean_sq_err < old_mean_sq_err);

  return make_tuple(mean_sq_err, iterations);
}

// ---------- kmeans_output ----------
// display results to standard output

template<typename CentroidsView>
void kmeans_output(int k_clusters, int point_count, int dimensions,
                   CentroidsView& centroids) {

  stapl::do_once([&](void) {
    cout << "k= " << k_clusters << ", n= " << point_count
         << ", d= " << dimensions << endl << "CENTROIDS" << endl;

    for (auto&& centroid : centroids) {
      for (auto&& coordinate : centroid) {
        cout << coordinate << " ";
      }
      cout << endl;
    }
  });
}
