/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cfloat>
#include <climits>
#include <cstdio>
#include <ctime>

#include <algorithm>
#include <iostream>
#include <fstream>
#include "confint.hpp"
#include <mpi.h>
using namespace std;

//#define CIO 1
#define TIMER 1

///////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans( int, int, int, FILE *, int, int );
void kmeans_input( int, int, int, int, FILE *, double ***, double **, int, int);
#else
void kmeans( int, int, int, ifstream&, int, int );
void kmeans_input( int, int, int, int, ifstream&, double ***, double **, int, int);
#endif

void kmeans_mpi( int, int, int, int *, double **, int *, double **,
                 double **, double *, int *);
void kmeans_output( int, int, int, double **, int);

inline double euclid_dist_sq(double x, double y) {
  double temp = x - y;
  return temp * temp;
}

///////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv )
{
  MPI_Init(&argc, &argv);
  int proc_count, my_id;
  MPI_Comm_size(MPI_COMM_WORLD, &proc_count);
  MPI_Comm_rank(MPI_COMM_WORLD, &my_id);

#ifdef STAPL_USE_PAPI
  if (PAPI_is_initialized()!=PAPI_LOW_LEVEL_INITED &&
      PAPI_library_init(PAPI_VER_CURRENT)!=PAPI_VER_CURRENT) {
    cerr << "PAPI initialization failed" << endl;
    exit(1);
  }
#endif

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
#ifdef CIO
  FILE *in_file = fopen( argv[4], "r" );
  if ( in_file == 0 ) {
    cerr << "Failed to open input file specified on command line" << endl;
    exit(1);
  }
#else
  ifstream in_str;
  in_str.open(argv[4],ios::binary|ios::in );
  if ( !in_str.is_open() ) {
    cerr << "Input file must be specified on command line" << endl;
    exit(1);
  }
#endif

#ifdef CIO
  kmeans( k_clusters, point_count, dimensions, in_file, my_id, proc_count );
  fclose(in_file);
#else
  kmeans( k_clusters, point_count, dimensions, in_str, my_id, proc_count );
  in_str.close();
#endif

  MPI_Finalize();
}

////////////////////////////////////////////////////////////////////////////////
// driver
////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans( int k_clusters, int point_count, int dimensions, FILE *in_file,
             int my_id, int proc_count )
#else
void kmeans( int k_clusters, int point_count, int dimensions, ifstream& in_str,
             int my_id, int proc_count )
#endif
{

#ifdef TIMER
  int *orig_assign_counts = new int[k_clusters];
  double ** orig_centroids = new double *[k_clusters];
  for ( int j=0; j<k_clusters; j++) {
    orig_centroids[j] = new double [dimensions];
  }

  counter_t timer;
  timer.reset();
  timer.start();
#endif

  int *old_assign_counts = new int[k_clusters];
  double ** old_centroids = new double *[k_clusters];
  for ( int j=0; j<k_clusters; j++) {
    old_centroids[j] = new double [dimensions];
  }
  int *new_assign_counts = new int[k_clusters];
  double ** new_centroids = new double *[k_clusters];
  for ( int j=0; j<k_clusters; j++) {
    new_centroids[j] = new double [dimensions];
  }

  double ** points;

  double mean_sq_err = DBL_MAX;
  int iterations = 0;

  // how many points to process?

  int my_point_count = 0;
  int block_size = point_count / proc_count;
  int big_blocks = point_count % block_size;
  if( my_id < big_blocks ) {
    my_point_count = block_size + 1;
  } else {
    my_point_count = block_size;
  }

#ifdef CIO
  kmeans_input( k_clusters, my_point_count, dimensions, big_blocks, in_file,
                &points, old_centroids, my_id, proc_count );
#else
  kmeans_input( k_clusters, my_point_count, dimensions, big_blocks, in_str,
                &points, old_centroids, my_id, proc_count );
#endif

#ifdef TIMER
  double io_time = timer.stop();

  for ( int j=0; j<k_clusters; j++) {
    orig_assign_counts[j] = old_assign_counts[j];
    for ( int d=0; d<dimensions; d++) {
      orig_centroids[j][d] = old_centroids[j][d];
    }
  }

  bool continue_iterating = true;
  confidence_interval_controller iter_control(32, 100, 0.05);
  while (continue_iterating)
  {
    mean_sq_err = DBL_MAX;
    for ( int j=0; j<k_clusters; j++) {
      old_assign_counts[j] = orig_assign_counts[j];
      for ( int d=0; d<dimensions; d++) {
        old_centroids[j][d] = orig_centroids[j][d];
      }
    }
    timer.reset();
    timer.start();
#endif

    kmeans_mpi( k_clusters, my_point_count, dimensions,
                old_assign_counts, old_centroids,
                new_assign_counts, new_centroids,
                points, &mean_sq_err, &iterations );

#ifdef TIMER
    iter_control.push_back(timer.stop());
    int local_iterate, iterate_sum(0);
    iter_control.iterate() ? local_iterate = 1 : local_iterate = 0;
    MPI_Allreduce(&local_iterate, &iterate_sum, 1,
        MPI_INT, MPI_SUM, MPI_COMM_WORLD);
    iterate_sum != 0 ? continue_iterating = true : continue_iterating = false;
  }

  for ( int j=0; j<k_clusters; j++) {
    delete[] orig_centroids[j];
  }
  delete[] orig_centroids;
#endif

  kmeans_output( k_clusters, point_count, dimensions, old_centroids, my_id );

#ifdef TIMER
  if( my_id == 0 ) {
    iter_control.report("km_mpi");
    cout << "io= " << io_time << ",";
    cout << "mse= " << mean_sq_err << ", iter= " << iterations << endl;
  }
#endif

  for ( int j=0; j<k_clusters; j++ ) {
    delete[] old_centroids[j];
    delete[] new_centroids[j];
  }
  for ( int j=0; j<my_point_count; j++ ) {
    delete[] points[j];
  }
  delete[] old_centroids;
  delete[] old_assign_counts;
  delete[] new_centroids;
  delete[] new_assign_counts;
  delete[] points;
}

////////////////////////////////////////////////////////////////////////////////
// read in the data points
// select initial cluster centroids
////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans_input( int k_clusters, int point_count, int dimensions,
                   int big_blocks, FILE *in_file, double ***points,
                   double **old_centroids, int my_id, int proc_count )
#else
void kmeans_input( int k_clusters, int point_count, int dimensions,
                   int big_blocks, ifstream &in_str, double ***points,
                   double **old_centroids, int my_id, int proc_count )
#endif
{

  *points = new double *[point_count];
  for ( int j=0; j<point_count; j++) {
    (*points)[j] = new double [dimensions];
  }

  // many processes reading the same input file at different offsets
  size_t offset = (point_count * my_id) * (dimensions * sizeof(double));
  if (big_blocks) {
    offset += my_id < big_blocks ?
      my_id * (dimensions * sizeof(double)) :
      big_blocks * (dimensions * sizeof(double));
  }
#ifdef CIO
  fseek( in_file, offset, SEEK_CUR );
  for ( int j=0; j<point_count; j++) {
    int count = fread( (*points)[j], sizeof(double), dimensions, in_file );
    if( count != dimensions ) {
      cerr << "Incomplete file read @ " << j << endl;
      exit(1);
    }
  }
#else
  in_str.seekg( offset );
  for ( int j=0; j<point_count; j++) {
    in_str.read( (char *) ( (*points)[j]), sizeof(double)*dimensions );
    if( in_str.fail() ) {
      cerr << "Incomplete file read @ " << j << endl;
      exit(1);
    }
  }
#endif

  int step = point_count / k_clusters;
  if (step == 0) {
    cerr << "Too few points on process 0 to select initial clusters" << endl;
    exit(1);
  }

  double *centroid_data = new double[k_clusters*dimensions];

  // Initial centroids are every step points.
  if ( !my_id ) {
    int next = 0;
    int linear_offset = 0;
    for ( int j=0; j<k_clusters; j++ ) {
      for ( int d=0; d<dimensions; d++, ++linear_offset ) {
        old_centroids[j][d] = (*points)[next][d];
        centroid_data[linear_offset] = (*points)[next][d];
      }
      next = ( next + step ) % point_count;
    }
  }

  // send the centroid selection to all processes
  MPI_Bcast( centroid_data, k_clusters*dimensions, MPI_DOUBLE, 0,
             MPI_COMM_WORLD );

  int linear_offset = 0;
  for ( int j=0; j<k_clusters; j++ ) {
    for ( int d=0; d<dimensions; d++, linear_offset++ ) {
      old_centroids[j][d] = centroid_data[linear_offset];
    }
  }
  delete[] centroid_data;
}

////////////////////////////////////////////////////////////////////////////////
// k-means computation
////////////////////////////////////////////////////////////////////////////////

void kmeans_mpi( int k_clusters, int point_count, int dimensions,
                 int *old_assign_counts, double **old_centroids,
                 int *new_assign_counts, double **new_centroids,
                 double **points, double *mean_sq_err, int *iterations ) {

  double *distance = new double [k_clusters];
  double old_mean_sq_err, new_mean_sq_err;

  do {
    old_mean_sq_err = *mean_sq_err;
    new_mean_sq_err = 0.0;

    // initialize for another iteration
    for ( int j=0; j<k_clusters; j++ ) {
      new_assign_counts[j] = 0;
    }
    for ( int j=0; j<k_clusters; j++ ) {
      for ( int d=0; d<dimensions; d++ ) {
        new_centroids[j][d] = 0.0;
      }
    }

    // for all points assigned to this process
    for ( int i=0; i<point_count; i++ ) {

      // compute squared Euclidean distance from all centroids to this point
      for ( int j=0; j<k_clusters; j++ ) {
        double dist = 0.0;
        for ( int d=0; d<dimensions; d++ ) {
          dist += euclid_dist_sq(points[i][d], old_centroids[j][d]);
        }
        distance[j] = dist;
      }

      // find the closest centroid to this point
      int nearest = -1;
      double min_val = std::numeric_limits<double>::max();
      for ( int j=0; j<k_clusters; j++ ) {
        if ( distance[j] < min_val ) {
          min_val = distance[j];
          nearest = j;
        }
      }

      // update the new owner centroid to include this point
      for ( int d=0; d<dimensions; d++ ) {
        new_centroids[nearest][d] += points[i][d];
      }
      new_assign_counts[nearest] = new_assign_counts[nearest] + 1;
      for ( int d=0; d<dimensions; d++ ) {
        new_mean_sq_err += euclid_dist_sq(points[i][d], old_centroids[nearest][d]);
      }
    }

    // re-estimate the centroids as the average of the assigned points
    for ( int j=0; j<k_clusters; j++ ) {
      MPI_Allreduce(&(new_assign_counts[j]), &(old_assign_counts[j]), 1,
        MPI_INT, MPI_SUM, MPI_COMM_WORLD);
      MPI_Allreduce(new_centroids[j], old_centroids[j], dimensions,
        MPI_DOUBLE, MPI_SUM, MPI_COMM_WORLD);
      old_assign_counts[j] = std::max(old_assign_counts[j], 1);
      for ( int d=0; d<dimensions; d++ ) {
        old_centroids[j][d] = old_centroids[j][d] / old_assign_counts[j];
      }
    }
    MPI_Allreduce(&new_mean_sq_err, mean_sq_err, 1, MPI_DOUBLE, MPI_SUM,
      MPI_COMM_WORLD);
    (*iterations)++;

  } while ( *mean_sq_err < old_mean_sq_err );
  delete[] distance;
}

////////////////////////////////////////////////////////////////////////////////
// display results to standard output
////////////////////////////////////////////////////////////////////////////////

void kmeans_output( int k_clusters, int point_count, int dimensions,
                    double **centroids, int my_id ) {

  if( my_id == 0 ) {
    cout << "k= " << k_clusters << ", n= " << point_count << ", ";
    cout << "d= " << dimensions << endl;

    cout << "CENTROIDS" << endl;
    for ( int j=0; j<k_clusters; j++ ) {
      for ( int d=0; d<dimensions; d++ ) {
        cout << centroids[j][d] << " ";
      }
      cout << endl;
    }
  }
}
