/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * bucketsort.c
 *
 * This file contains code that implement a variety of counting sorting
 * algorithms
 *
 * Started 7/25/97
 * George
 *
 */

#include <metis.h>



/*************************************************************************
* This function uses simple counting sort to return a permutation array
* corresponding to the sorted order. The keys are assumed to start from
* 0 and they are positive.  This sorting is used during matching.
**************************************************************************/
void BucketSortKeysInc(int n, int max, idxtype *keys, idxtype *tperm, idxtype *perm)
{
  int i, ii;
  idxtype *counts;

  counts = idxsmalloc(max+2, 0, "BucketSortKeysInc: counts");

  for (i=0; i<n; i++)
    counts[keys[i]]++;
  MAKECSR(i, max+1, counts);

  for (ii=0; ii<n; ii++) {
    i = tperm[ii];
    perm[counts[keys[i]]++] = i;
  }

  free(counts);
}

