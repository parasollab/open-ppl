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
using namespace std;

///////////////////////////////////////////////////////////////////////////

void kmeans( int, int, int, ifstream&, int);
void kmeans_input( int, int, int, 
                   ifstream&, double ***, double **, int);

void kmeans_nr( int, int, int, int *, double **, 
                    double **, int *, int *);
void kmeans_output( int, int, int, double **);

///////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv ) {

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
  in_str.open(argv[4],ios::binary|ios::in );
  if ( !in_str.is_open() ) {
    cerr << "Input file must be specified on command line" << endl;
    exit(1);
  }

  int seed = 0;
  if ( argc < 6 ) {
    time_t timer;
    time(&timer);
    struct tm *tm = localtime(&timer);
    seed = (1+tm->tm_sec) * (1+tm->tm_min) * (1+tm->tm_hour) * tm->tm_mday;
  } else {
    seed = atoi( argv[5] );
  }
  srand(seed);
  int first = rand() % point_count;

  kmeans( k_clusters, point_count, dimensions, in_str, first);
  in_str.close();
}

////////////////////////////////////////////////////////////////////////////////
// driver
////////////////////////////////////////////////////////////////////////////////

void kmeans( int k_clusters, int point_count, int dimensions, 
             ifstream& in_str, int first)
{

  int *assign_index = new int[point_count];
  int *assign_counts = new int[k_clusters];
  double ** centroids = new double *[k_clusters];
  for ( int j=0; j<k_clusters; j++) {
    centroids[j] = new double [dimensions];
  }

  double ** points;

  int iterations = 0;

  kmeans_input( k_clusters, point_count, dimensions, 
                in_str, &points, centroids, first);

  kmeans_nr( k_clusters, point_count, dimensions,
                assign_counts, centroids,
                points, assign_index, &iterations );

  kmeans_output( k_clusters, point_count, dimensions, centroids );

}

////////////////////////////////////////////////////////////////////////////////
// read in the data points
// select initial cluster centroids
////////////////////////////////////////////////////////////////////////////////

void kmeans_input( int k_clusters, int point_count, int dimensions, 
                   ifstream &in_str, double ***points, double **centroids,
                   int first)
{
  *points = new double *[point_count];
  for ( int j=0; j<point_count; j++) {
    (*points)[j] = new double [dimensions];
  }

  for ( int j=0; j<point_count; j++) {
    in_str.read( (char *)(*points)[j], sizeof(double)*dimensions );
    if( in_str.fail() ) {
      cerr << "Incomplete file read @ " << j << endl;
      exit(1);
    }
  }

  int step = point_count / k_clusters;
  int next = first;
  for ( int j=0; j<k_clusters; j++ ) {
    for ( int d=0; d<dimensions; d++ ) {
      centroids[j][d] = (*points)[next][d];
    }
    next = ( next + step ) % point_count;
    
  }
}

////////////////////////////////////////////////////////////////////////////////
// k-means computation
////////////////////////////////////////////////////////////////////////////////

void kmeans_nr( int k_clusters, int point_count, int dimensions,
                int *assign_counts, double **centroids,     
                double ** points, int * assign_index, int *iterations ) {

  int change = 0;
  do {

    // E Step

    change = 0;
    for ( int cl_ndx=0; cl_ndx<k_clusters; cl_ndx++) {
      assign_counts[cl_ndx] = 0;
    }

    for ( int pt_ndx=0; pt_ndx<point_count; pt_ndx++) {
      double dist_min = DBL_MAX;
      int cen_min = -1;
      for ( int cl_ndx=0; cl_ndx<k_clusters; cl_ndx++) {
        double dist = 0.0;
        for ( int d=0; d<dimensions; d++) {
          double temp = points[pt_ndx][d]- centroids[cl_ndx][d];
          dist += temp * temp;
        }
        if ( dist < dist_min) {
          dist_min = dist;
          cen_min = cl_ndx;
        }
      }
      if ( cen_min != assign_index[pt_ndx] ) {
         change++ ;
      }
      assign_counts[cen_min]++;
      assign_index[pt_ndx] = cen_min;
    }

    // M step

    for ( int cl_ndx=0; cl_ndx<k_clusters; cl_ndx++) {
      for ( int d=0; d<dimensions; d++)  {
        centroids[cl_ndx][d] = 0.0;
      }
    }
    for ( int pt_ndx=0; pt_ndx<point_count; pt_ndx++) {
      for (int d=0; d<dimensions; d++) {
        centroids[assign_index[pt_ndx]][d] += points[pt_ndx][d];
      }
    }
    for ( int cl_ndx=0; cl_ndx<k_clusters ; cl_ndx++) {
      if ( assign_counts[cl_ndx] > 0) {
        double temp = 1.0 / assign_counts[cl_ndx];
        for ( int d=0; d<dimensions; d++ ) {
          centroids[cl_ndx][d] *= temp;
        }
      }
    }
    (*iterations)++;
  } while( change && (*iterations) < 20  );
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
