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

using namespace std;

//#define CIO 1
#define TIMER 1

////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans( int, int, int, FILE * );
void kmeans_input( int, int, int, FILE *, double **, double **);
#else
void kmeans( int, int, int, ifstream & );
void kmeans_input( int, int, int, ifstream &, double **, double **);
#endif

void kmeans_seq( int, int, int, int *, double **, int *, double **,
                 double **, double *, int * );
void kmeans_output( int, int, int, double **);

inline double euclid_dist_sq(double x, double y) {
  double temp = x - y;
  return temp * temp;
}

////////////////////////////////////////////////////////////////////////////////

int main( int argc, char **argv)
{
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
  in_str.open(argv[4], ios::binary|ios::in );
  if ( !in_str.is_open() ) {
    cerr << "Input file must be specified on command line" << endl;
    exit(1);
  }
#endif

#if CIO
  kmeans( k_clusters, point_count, dimensions, in_file );
  fclose(infile);
#else
  kmeans( k_clusters, point_count, dimensions, in_str );
  in_str.close();
#endif

}

////////////////////////////////////////////////////////////////////////////////
// driver
////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans( int k_clusters, int point_count, int dimensions, FILE *in_file )
#else
void kmeans( int k_clusters, int point_count, int dimensions, ifstream& in_str )
#endif
{

#ifdef TIMER
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

  double ** points = new double *[point_count];
  for ( int j=0; j<point_count; j++) {
    points[j] = new double [dimensions];
  }

  double mean_sq_err = DBL_MAX;
  int iterations = 0;

#ifdef CIO
  kmeans_input( k_clusters, point_count, dimensions, in_file,
                points, old_centroids );
#else

  kmeans_input( k_clusters, point_count, dimensions, in_str,
                points, old_centroids );
#endif

#ifdef TIMER
  double io_time = timer.stop();

 double ** initial_centroids = new double *[k_clusters];
 for ( int j=0; j<k_clusters; j++) {
   initial_centroids[j] = new double [dimensions];
   for ( int d=0; d<dimensions; d++ ) {
     initial_centroids[j][d] = old_centroids[j][d];
   }
 }

  confidence_interval_controller iter_control(32, 100, 0.05);
  while (iter_control.iterate())
  {
    for ( int j=0; j<k_clusters; j++ ) {
      for ( int d=0; d<dimensions; d++ ) {
        old_centroids[j][d] = initial_centroids[j][d];
      }
    }

    timer.reset();
    timer.start();
#endif

  kmeans_seq( k_clusters, point_count, dimensions, 
              old_assign_counts, old_centroids, 
              new_assign_counts, new_centroids, 
              points, &mean_sq_err, &iterations );

#ifdef TIMER
    iter_control.push_back(timer.stop());
  }
#endif

  kmeans_output( k_clusters, point_count, dimensions, old_centroids );
              
#ifdef TIMER
 iter_control.report("km_seq");
 cout << "io= " << io_time << ",";
 cout << "mse= " << mean_sq_err << ", iter= " << iterations << endl;
#endif
}

////////////////////////////////////////////////////////////////////////////////
// read in the data points
// select initial cluster centroids 
////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans_input( int k_clusters, int point_count, int dimensions, 
                   FILE *in_file, double **points, double **old_centroids )
#else
void kmeans_input( int k_clusters, int point_count, int dimensions, 
                   ifstream &in_str, double **points, double **old_centroids )
#endif
{

#ifdef CIO
  for ( int j=0; j<point_count; j++) {
    int count = fread( points[j], sizeof(double), dimensions, in_file );
    if( count != dimensions ) {
      cerr << "Incomplete file read @ " << j << endl;
      exit(1);
    }
  }
#else
  for ( int j=0; j<point_count; j++) {
    in_str.read( (char *)(points[j]), sizeof(double)*dimensions );
    if( in_str.fail() ) {
      cerr << "Incomplete file read @ " << j << endl;
      exit(1);
    }
  }
#endif

  int step = point_count / k_clusters;
  int next = 0;
  for ( int j=0; j<k_clusters; j++ ) {
    for ( int d=0; d<dimensions; d++ ) {
      old_centroids[j][d] = points[next][d];
    }
    next = ( next + step ) % point_count;
  }
}

////////////////////////////////////////////////////////////////////////////////
// k-means computation
////////////////////////////////////////////////////////////////////////////////

void kmeans_seq( int k_clusters, int point_count, int dimensions, 
                 int *old_assign_counts, double **old_centroids, 
                 int *new_assign_counts, double **new_centroids, 
                 double **points, double *mean_sq_err, int *iterations ) {

  double * distance = new double [k_clusters];
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

    // for all points
    for ( int i=0; i<point_count; i++ ) {

      // compute squared Euclidean distance from all centroids to this point
      for ( int j=0; j<k_clusters; j++ ) {
        double dist = 0.0;
        for ( int d=0; d<dimensions; d++ ) {
          dist += euclid_dist_sq(points[i][d], old_centroids[j][d]);
        }
        distance[j] = dist;
      }

      // find the nearest centroid to this point
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
      old_assign_counts[j] = std::max(new_assign_counts[j], 1);
      for ( int d=0; d<dimensions; d++ ) {
        old_centroids[j][d] = new_centroids[j][d] / old_assign_counts[j];
      }
    }
    (*iterations)++;

    *mean_sq_err = new_mean_sq_err;
  } while (*mean_sq_err < old_mean_sq_err );
}

////////////////////////////////////////////////////////////////////////////////
// display results to standard output
////////////////////////////////////////////////////////////////////////////////

void kmeans_output( int k_clusters, int point_count, int dimensions, 
                    double **centroids ) {

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
