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

#ifdef CIO
void kmeans( int, int, int, FILE *, int, int, int );
void kmeans_input( int, int, int, FILE *, double ****, int *, int *, double **, int, int, int);
#else
void kmeans( int, int, int, ifstream&, int, int, int );
void kmeans_input( int, int, int, ifstream&, double ****, int *, int *, double **, int, int, int);
#endif

void kmeans_mpi( int, int, int, int *, double **, int *, double **,
                 double **, double *, int *, int);

void kmeans_mpi( int, int, int *, double **, int *, double **, double ***,
                 int, int *, double *, int *);

void kmeans_output( int, int, int, double **, int);

inline double euclid_dist_sq(double x, double y) {
  double temp = x - y;
  return temp * temp;
}

///////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv ) {

  MPI_Init(&argc, &argv);
  int proc_count, my_id;
  MPI_Comm_size(MPI_COMM_WORLD, &proc_count);
  MPI_Comm_rank(MPI_COMM_WORLD, &my_id);

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
  if ( argc < 6 ) {
    cerr << "Must specify block size " << endl; 
    exit(1);
  }
  int block_size = atoi( argv[5] );

#ifdef CIO
  kmeans( k_clusters, point_count, dimensions, in_file, my_id, proc_count, block_size );
  fclose(in_file);
#else
  kmeans( k_clusters, point_count, dimensions, in_str, my_id, proc_count, block_size );
  in_str.close();
#endif


  MPI_Finalize();
}

////////////////////////////////////////////////////////////////////////////////
// driver
////////////////////////////////////////////////////////////////////////////////

#ifdef CIO
void kmeans( int k_clusters, int point_count, int dimensions, FILE *in_file,
             int my_id, int proc_count, int block_size )
#else
void kmeans( int k_clusters, int point_count, int dimensions, ifstream& in_str,
             int my_id, int proc_count, int block_size )
#endif
{

#ifdef TIMER
  counter_t timer;
  timer.reset();
  timer.start();
#endif

  int *orig_assign_counts = new int[k_clusters];
  double ** orig_centroids = new double *[k_clusters];
  for ( int j=0; j<k_clusters; j++) {
    orig_centroids[j] = new double [dimensions];
  }

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

  double *** point_blocks;
  int my_num_blocks;
  int * my_block_sizes = new int[2];

  double mean_sq_err = DBL_MAX;
  int iterations = 0;

#ifdef CIO
  kmeans_input( k_clusters, point_count, dimensions, in_file, &point_blocks, &my_num_blocks,
                my_block_sizes, old_centroids, my_id, proc_count, block_size );
#else
  kmeans_input( k_clusters, point_count, dimensions, in_str, &point_blocks, &my_num_blocks,
                my_block_sizes, old_centroids, my_id, proc_count, block_size );
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

    kmeans_mpi( k_clusters, dimensions, old_assign_counts, old_centroids,
                new_assign_counts, new_centroids, point_blocks, my_num_blocks,
                my_block_sizes, &mean_sq_err, &iterations);

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
}

////////////////////////////////////////////////////////////////////////////////
// read in the data points
// select initial cluster centroids
////////////////////////////////////////////////////////////////////////////////
#ifdef CIO
void kmeans_input( int k_clusters, int point_count, int dimensions, FILE *in_file,
                   double ****point_blocks, int* my_num_blocks, int * my_block_sizes,
                   double **old_centroids, int my_id, int proc_count, int block_size)
#else
void kmeans_input( int k_clusters, int point_count, int dimensions, ifstream &in_str,
                   double ****point_blocks, int* my_num_blocks, int * my_block_sizes,
                   double **old_centroids, int my_id, int proc_count, int block_size)
#endif
{
  int global_num_blocks  = point_count / block_size;
  global_num_blocks     += point_count % block_size ? 1 : 0;
  *my_num_blocks         = global_num_blocks / proc_count;

  my_block_sizes[0] = block_size;

  // There's a partial row and I have a block on it
  if (my_id < global_num_blocks % proc_count)
    ++(*my_num_blocks);

  // check if my process last block and determine its size.
  if (((global_num_blocks % proc_count == 0) && (my_id == proc_count - 1))
      || my_id == global_num_blocks % proc_count-1) {
      my_block_sizes[1] = (point_count % block_size == 0)
                          ? block_size : point_count % block_size;
  }
  // if not last block, set my second block size to default. 
  else {
    my_block_sizes[1] = block_size;
  }
    
  *point_blocks = new double **[*my_num_blocks];
  for (int i=0; i< *my_num_blocks; i++) {

    int point_count = (i != *my_num_blocks-1)
                      ? my_block_sizes[0] : my_block_sizes[1];

    (*point_blocks)[i] = new double *[point_count];
    
    for (int j=0; j < point_count; ++j) {
      (*point_blocks)[i][j] = new double [dimensions];
    }
  }

  for (int i=0; i< *my_num_blocks; i++) {
    int read_point_count = (i != *my_num_blocks-1)
                           ? my_block_sizes[0] : my_block_sizes[1];
    // num blocks before me.
    size_t offset = ((i * proc_count + my_id) * block_size) * dimensions * sizeof(double);
  
    in_str.seekg(offset);
    for (int j=0; j<read_point_count; j++) {
       in_str.read( (char *) ((*point_blocks)[i][j]), sizeof(double) * dimensions );
       if( in_str.fail() ) {
        cerr << "Incomplete file read @ block=" << i << " / elem= " << j << endl;
        exit(1);
      }
    }
  }

  int num_sample_blocks = my_block_sizes[0] == my_block_sizes[1] 
                          ? *my_num_blocks : *my_num_blocks-1; 

  if (num_sample_blocks == 0) {
    cerr << "No full blocks on process 0 to select initial clusters" << endl;
    exit(1);
  } 

  int elems_per_block = k_clusters / num_sample_blocks;
  
  if (( elems_per_block + k_clusters % num_sample_blocks)  > block_size) {
    cerr << "Too few points on process 0 to select initial clusters" << endl;
    exit(1);
  }

  double *centroid_data = new double[k_clusters*dimensions];

  // Initial centroids by selecting points from each full block
  if ( !my_id ) {
    int offset        = 0;
    int linear_offset = 0;

    for (int i=0; i<num_sample_blocks && offset < k_clusters; i++) {

      int elems_from_this_block = elems_per_block;
      elems_from_this_block    += i==0 ? k_clusters % num_sample_blocks : 0;
      
      for (int j=0; j<elems_from_this_block && offset < k_clusters; j++, offset++) {

        for ( int d=0; d<dimensions; d++, ++linear_offset ) {
          old_centroids[offset][d]     = (*point_blocks)[i][j][d];
          centroid_data[linear_offset] = (*point_blocks)[i][j][d];
        }
      }
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

void kmeans_mpi( int k_clusters, int dimensions,
                 int *old_assign_counts, double **old_centroids,
                 int *new_assign_counts, double **new_centroids,
                 double ***point_blocks, int num_blocks, int *block_sizes,
                 double *mean_sq_err, int *iterations) {

  double *distance = new double [k_clusters];
  double old_mean_sq_err, new_mean_sq_err;
  *iterations = 0;
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

    // for all point blocks assigned to this process
    for ( int k=0; k<num_blocks; ++k) {

      // possibly small blocks on last row blocks..
      const int point_count = (k != num_blocks-1) ? block_sizes[0] : block_sizes[1];

      // for all points in this block
      for ( int i=0; i<point_count; i++ ) {

        // compute squared Euclidean distance from all centroids to this point
        for ( int j=0; j<k_clusters; j++ ) {
          double dist = 0.0;
          for ( int d=0; d<dimensions; d++ ) {
            dist += euclid_dist_sq(point_blocks[k][i][d], old_centroids[j][d]);
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
          new_centroids[nearest][d] += point_blocks[k][i][d];
        }
        new_assign_counts[nearest] = new_assign_counts[nearest] + 1;
        for ( int d=0; d<dimensions; d++ ) {
          new_mean_sq_err += euclid_dist_sq(point_blocks[k][i][d], old_centroids[nearest][d]);
        }
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
