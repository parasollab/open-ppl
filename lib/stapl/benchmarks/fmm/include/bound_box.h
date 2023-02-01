/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_BOUND_BOX_H
#define STAPL_BENCHMARKS_FMM_BOUND_BOX_H

#include "logger.h"
#include "thread.h"
#include "types.h"

class BoundBox
{
private:
  /// Threshold of NBODY for spawning new threads
  const int nspawn;

  /// Recursive functor for calculating bounds of bodies
  struct BodiesRecursion
  {
    /// Begin iterator of bodies
    B_iter BiBegin;
    /// End iterator of bodies
    B_iter BiEnd;
    /// Bounds : Contains Xmin, Xmax
    Bounds & bounds;
    /// Threshold of NBODY for spawning new threads
    int nspawn;
    BodiesRecursion(B_iter a_BiBegin,
                    B_iter a_BiEnd,
                    Bounds & a_bounds, int a_nspawn)
      : BiBegin(a_BiBegin),
        BiEnd(a_BiEnd),
        bounds(a_bounds),
        nspawn(a_nspawn)
    { }

    void operator()()
    {
      // Validate range
      assert(BiEnd - BiBegin > 0);
      // If number of elements is small enough
      if (BiEnd - BiBegin < nspawn) {
        // Loop over range of bodies
        for (B_iter B=BiBegin; B!=BiEnd; B++) {
          // Update Xmin
          bounds.Xmin = min(B->X, bounds.Xmin);
          // Update Xmax
          bounds.Xmax = max(B->X, bounds.Xmax);
        }
      }
      // Else if number of elements are large
      else {
        // Middle iterator
        B_iter BiMid = BiBegin + (BiEnd - BiBegin) / 2;
        // Copy bounds
        Bounds bounds2 = bounds;
        // Initialize tasks
        mk_task_group;
        // Instantiate recursive functor
        BodiesRecursion leftBranch(BiBegin, BiMid, bounds, nspawn);
        // Create new task for left branch
        create_taskc(leftBranch);
        // Instantiate recursive functor
        BodiesRecursion rightBranch(BiMid, BiEnd, bounds2, nspawn);
        // Use old task for right branch
        rightBranch();
        // Synchronize tasks
        wait_tasks;
        // Minimum of the two Xmins
        bounds.Xmin = min(bounds.Xmin, bounds2.Xmin);
        // Maximum of the two Xmaxs
        bounds.Xmax = max(bounds.Xmax, bounds2.Xmax);
      }
    }
  };

  /// Recursive functor for calculating bounds of cells
  struct CellsRecursion
  {
    /// Begin iterator of cells
    C_iter CiBegin;
    /// End iterator of cells
    C_iter CiEnd;
    /// Bounds : Contains Xmin, Xmax
    Bounds & bounds;
    /// Threshold of NBODY for spawning new threads
    int nspawn;
    CellsRecursion(C_iter a_CiBegin,
                   C_iter a_CiEnd,
                   Bounds & a_bounds,
                   int a_nspawn)
      : CiBegin(a_CiBegin),
        CiEnd(a_CiEnd),
        bounds(a_bounds),
        nspawn(a_nspawn)
    { }

    void operator()()
    {
      // Validate range
      assert(CiEnd - CiBegin > 0);
      // If number of elements is small enough
      if (CiEnd - CiBegin < nspawn) {
        // Loop over range of cells
        for (C_iter C=CiBegin; C!=CiEnd; C++) {
          // Update Xmin
          bounds.Xmin = min(C->X, bounds.Xmin);
          // Update Xmax
          bounds.Xmax = max(C->X, bounds.Xmax);
        }
      }
      //  Else if number of elements are large
      else {
        // Middle iterator
        C_iter CiMid = CiBegin + (CiEnd - CiBegin) / 2;
        // Copy bounds
        Bounds bounds2 = bounds;
        // Initialize tasks
        mk_task_group;
        // Instantiate recursive functor
        CellsRecursion leftBranch(CiBegin, CiMid, bounds, nspawn);
        // Create new task for left branch
        create_taskc(leftBranch);
        // Instantiate recursive functor
        CellsRecursion rightBranch(CiMid, CiEnd, bounds2, nspawn);
        // Use old task for right branch
        rightBranch();
        // Synchronize tasks
        wait_tasks;
        // Minimum of the two Xmins
        bounds.Xmin = min(bounds.Xmin, bounds2.Xmin);
        // Maximum of the two Xmaxs
        bounds.Xmax = max(bounds.Xmax, bounds2.Xmax);
      }
    }
  };

public:
  BoundBox(int a_nspawn)
    : nspawn(a_nspawn)
  { }

  /// Get Xmin and Xmax of bodies
  Bounds getBounds(Bodies & bodies)
  {
    // Start timer
    logger::startTimer("Get bounds");
    // Bounds : Contains Xmin, Xmax
    Bounds bounds;
    // If body vector is empty set bounds to 0
    if (bodies.empty()) {
      bounds.Xmin = bounds.Xmax = 0;
    }
    // If body vector is not empty
    else {
      // Initialize Xmin, Xmax
      bounds.Xmin = bounds.Xmax = bodies.front().X;
      // Instantiate recursive functor
      BodiesRecursion bodiesRecursion(bodies.begin(), bodies.end(),
                                      bounds, nspawn);
      // Recursive call for bounds calculation
      bodiesRecursion();
    }
    // Stop timer
    logger::stopTimer("Get bounds");
    // Return Xmin and Xmax
    return bounds;
  }

  /// Update Xmin and Xmax of bodies
  Bounds getBounds(Bodies bodies, Bounds bounds)
  {
    // Start timer
    logger::startTimer("Get bounds");
    // Instantiate recursive functor
    BodiesRecursion bodiesRecursion(bodies.begin(),bodies.end(),bounds,nspawn);
    // Recursive call for bounds calculation
    bodiesRecursion();
    // Stop timer
    logger::stopTimer("Get bounds");
    // Return Xmin and Xmax
    return bounds;
  }

  /// Get Xmin and Xmax of cells
  Bounds getBounds(Cells cells)
  {
    // Start timer
    logger::startTimer("Get bounds");
    // Bounds : Contains Xmin, Xmax
    Bounds bounds;
    // If cell vector is empty set bounds to 0
    if (cells.empty()) {
      bounds.Xmin = bounds.Xmax = 0;
    }
    // If cell vector is not empty
    else {
      // Initialize Xmin, Xmax
      bounds.Xmin = bounds.Xmax = cells.front().X;
      // Instantiate recursive functor
      CellsRecursion cellsRecursion(cells.begin(),cells.end(),bounds,nspawn);
      //  Recursive call for bounds calculation
      cellsRecursion();
    }
    // Stop timer
    logger::stopTimer("Get bounds");
    // Return Xmin and Xmax
    return bounds;
  }

  /// Update Xmin and Xmax of cells
  Bounds getBounds(Cells cells, Bounds bounds)
  {
    // Start timer
    logger::startTimer("Get bounds");
    // Instantiate recursive functor
    CellsRecursion cellsRecursion(cells.begin(),cells.end(),bounds,nspawn);
    // Recursive call for bounds calculation
    cellsRecursion();
    // Stop timer
    logger::stopTimer("Get bounds");
    // Return Xmin and Xmax
    return bounds;
  }
};

#endif // STAPL_BENCHMARKS_FMM_BOUND_BOX_H
