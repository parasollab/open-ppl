/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_SORT_H
#define STAPL_BENCHMARKS_FMM_SORT_H

#include "types.h"
#ifndef _OPENMP
int omp_get_num_threads() {return 1;}
int omp_get_thread_num() {return 0;}
#else
#include <omp.h>
#endif

/// Custom radix sort for body and structures
class Sort
{
private:
  /// Output buffer
  Bodies output;

private:
  /// Radixsorts the values using the keys
  void radixsort(int * key, int * value, int size)
  {
    // Number of bits in one stride
    const int bitStride = 8;
    // Size of stride in decimal
    const int stride = 1 << bitStride;
    // Mask the bits in one stride
    const int mask = stride - 1;
    // Number of OpenMP threads
    int numThreads;
    // Maximum value of key
    int maxKey = 0;
    // Bucket for each thread
    int (*bucketPerThread)[stride];
    // MaxKey for each thread
    int * maxKeyPerThread;
    // Buffer for both key and value
    int * buffer = new int [size];
    // Permutation index
    int * permutation = new int [size];
#ifdef _OPENMP
#pragma omp parallel
#endif
    // Start OpenMP parallel clause
    {
      // Get number of OpenMP threads
      numThreads = omp_get_num_threads();
#ifdef _OPENMP
#pragma omp single
#endif
      // Start serial clause
      {
        // Allocate bucket per thread
        bucketPerThread = new int [numThreads][stride]();
        // Allocate maxKey per thread
        maxKeyPerThread = new int [numThreads];
        // Loop over threads
        for (int i=0; i<numThreads; i++)
          // Initialize maxKey per thread
          maxKeyPerThread[i] = 0;
      // End serial clause
      }
#ifdef _OPENMP
#pragma omp for
#endif
      // Loop over keys
      for ( int i=0; i<size; i++ )
        // If key is larger than maxKey
        if ( key[i] > maxKeyPerThread[omp_get_thread_num()] )
          // Update maxKey per thread
          maxKeyPerThread[omp_get_thread_num()] = key[i];
#ifdef _OPENMP
#pragma omp single
#endif
      // Loop over threads
      for ( int i=0; i<numThreads; i++ )
        // Update maxKey
        if ( maxKeyPerThread[i] > maxKey ) maxKey = maxKeyPerThread[i];
          // While there are bits in maxKey to process
          while ( maxKey > 0 ) {
            // Initialize bucket
            int bucket[stride] = {0};
#ifdef _OPENMP
#pragma omp single
#endif
            // Loop over threads
            for ( int t=0; t<numThreads; t++ )
              // Loop over strides
              for ( int i=0; i<stride; i++ )
                // Initialize bucket per thread
                bucketPerThread[t][i] = 0;
#ifdef _OPENMP
#pragma omp for
#endif
            // Loop over keys
            for ( int i=0; i<size; i++ )
              // Increment bucket
              bucketPerThread[omp_get_thread_num()][key[i] & mask]++;
#ifdef _OPENMP
#pragma omp single
#endif
            // Start serial clause
            {
              // Loop over threads
              for ( int t=0; t<numThreads; t++ )
                // Loop over strides
                for ( int i=0; i<stride; i++ )
                  // Update bucket from all threads
                  bucket[i] += bucketPerThread[t][i];
              // Loop over strides
              for ( int i=1; i<stride; i++ )
                // Scan bucket over strides
                bucket[i] += bucket[i-1];
              // Loop over keys backwards
              for ( int i=size-1; i>=0; i-- )
                // Reverse scan bucket to get permutation
                permutation[i] = --bucket[key[i] & mask];
            // End serial clause
            }
#ifdef _OPENMP
#pragma omp for
#endif
            // Loop over values
            for ( int i=0; i<size; i++ )
              // Sort into buffer
              buffer[permutation[i]] = value[i];
#ifdef _OPENMP
#pragma omp for
#endif
            // Loop over values
            for ( int i=0; i<size; i++ )
              // Copy back from buffer
              value[i] = buffer[i];
#ifdef _OPENMP
#pragma omp for
#endif
            // Loop over keys
            for ( int i=0; i<size; i++ )
              // Sort into buffer
              buffer[permutation[i]] = key[i];
#ifdef _OPENMP
#pragma omp for
#endif
            // Loop over keys
            for ( int i=0; i<size; i++ )
              // Copy back from buffer and bit shift keys
              key[i] = buffer[i] >> bitStride;
#ifdef _OPENMP
#pragma omp single
#endif
            // Bit shift maxKey
            maxKey >>= bitStride;
                // End while for bits to process
          }
        // End OpenMP parallel clause
        }
    // Deallocate bucket per thread
    delete[] bucketPerThread;
    // Deallocate maxKey per thread
    delete[] maxKeyPerThread;
    // Deallocate buffer
    delete[] buffer;
    // Deallocate permutation index
    delete[] permutation;
  }

public:
  /// Sort input according to ibody
  Bodies ibody(Bodies & input)
  {
    // Size of bodies vector
    const int size = input.size();
    // Allocate key array
    int * key = new int [size];
    // Allocate index array
    int * index = new int [size];
    // Loop over input bodies
    for (B_iter B=input.begin(); B!=input.end(); B++) {
      // Body index
      int i = B-input.begin();
      // Copy IBODY to key array
      key[i] = B->IBODY;
      // Initialize index array
      index[i] = i;
    // End loop over input bodies
    }
    // Radix sort index according to key
    radixsort(key,index,size);
    // Resize output buffer
    output.resize(size);
    // Loop over output bodies
    for (B_iter B=output.begin(); B!=output.end(); B++) {
      // Body index
      int i = B-output.begin();
      // Permute according to index
      *B = input[index[i]];
    // End loop over output bodies
    }
    // Deallocate key array
    delete[] key;
    // Deallocate index array
    delete[] index;
    // Return output
    return output;
  }

  /// Sort input according to irank
  Bodies irank(Bodies & input)
  {
    // Size of bodies vector
    const int size = input.size();
    // Allocate key array
    int * key = new int [size];
    // Allocate index array
    int * index = new int [size];
    // Loop over input bodies
    for (B_iter B=input.begin(); B!=input.end(); B++) {
      // Body index
      int i = B-input.begin();
      // Copy IRANK to key array
      key[i] = B->IRANK;
      // Initialize index array
      index[i] = i;
    // End loop over input bodies
    }
    // Radix sort index according to key
    radixsort(key,index,size);
    // Resize output buffer
    output.resize(size);
    // Loop over output bodies
    for (B_iter B=output.begin(); B!=output.end(); B++) {
      // Body index
      int i = B-output.begin();
      // Permute according to index
      *B = input[index[i]];
    // End loop over output bodies
    }
    // Deallocate key array
    delete[] key;
    // Deallocate index array
    delete[] index;
    // Return output
    return output;
  }

  /// Sort bodies back to original order
  Bodies unsort(Bodies & bodies)
  {
    // Sort bodies
    bodies = ibody(bodies);
    return bodies;
  }
};

#endif // STAPL_BENCHMARKS_FMM_SORT_H
