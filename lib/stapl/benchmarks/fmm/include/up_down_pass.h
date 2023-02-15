/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_UP_DOWN_PASS_H
#define STAPL_BENCHMARKS_FMM_UP_DOWN_PASS_H

#include "kernel.h"
#include "logger.h"
#include "thread.h"

class UpDownPass
{
private:
  /// Multipole acceptance criteria
  const real_t theta;
  /// Use maximum distance for MAC
  const bool useRmax;
  /// Use error optimized theta for MAC
  const bool useRopt;

private:
  /// Recursive functor for error optimization of R
  struct SetRopt
  {
    /// Iterator of current cell
    C_iter C;
    /// Iterator of first cell
    C_iter C0;
    /// Root coefficient
    real_t c;
    /// Multipole acceptance criteria
    real_t theta;
    SetRopt(C_iter a_C, C_iter a_C0, real_t a_c, real_t a_theta)
      : C(a_C), C0(a_C0), c(a_c), theta(a_theta)
    { }

    void operator()()
    {
      // Initialize tasks
      mk_task_group;
      // Loop over child cells
      for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++) {
        // Initialize recusive functor
        SetRopt setRopt(CC, C0, c, theta);
        // Create new task for recursive call
        create_taskc(setRopt);
      }
      // Synchronize tasks
      wait_tasks;
#if MASS
      // Normalize multipole expansion coefficients
      for (int i=1; i<NTERM; i++) C->M[i] /= C->M[0];
#endif
      // Inverse of theta
      real_t x = 1.0 / theta;
      // Newton-Raphson won't work for theta==1
      assert(theta != 1.0);
      // Cell coefficient
      real_t a = c * powf(std::abs(C->M[0]),1.0/3);
      // Loop for Newton-Raphson iteration
      for (int i=0; i<5; i++) {
        // Function value
        real_t f = x * x - 2 * x + 1 - a * std::pow(x,-P);
        // Function derivative value
        real_t df = (P + 2) * x - 2 * (P + 1) + P / x;
        // Increment x
        x -= f / df;
      }
      // Multiply R by error optimized parameter x
      C->R *= x * theta;
    }
  };

  /// Recursive functor for the post-order traversal during upward pass
  struct PostOrderTraversal {
    /// Iterator of current cell
    C_iter C;
    /// Iterator of first cell
    C_iter C0;
    /// Multipole acceptance criteria
    real_t theta;
    /// Use maximum distance for MAC
    bool useRmax;

    /// Redefine cell radius R based on maximum distance
    void setRmax()
    {
      // Initialize Rmax
      real_t Rmax = 0;
      // If leaf cell
      if (C->NCHILD == 0) {
        // Loop over bodies in cell
        for (B_iter B=C->BODY; B!=C->BODY+C->NBODY; B++) {
          // Distance vector from particles to center of expansion
          vec3 dX = C->X - B->X;
          // Scalar distance
          real_t R = std::sqrt(norm(dX));
          // Maximum distance
          if (R > Rmax) Rmax = R;
        }
      }
      // If not leaf cell
      else {
        // Loop over child cells
        for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++) {
          // Distance vector from particles to center of expansion
          vec3 dX = C->X - CC->X;
          // Scalar distance
          real_t R = std::sqrt(norm(dX)) + CC->R;
          // Maximum distance
          if (R > Rmax) Rmax = R;
        }
      }
      // Redefine R based on maximum distance
      C->R = std::min(C->R,Rmax);
    }

    PostOrderTraversal(C_iter a_C, C_iter a_C0, real_t a_theta, bool a_useRmax)
      : C(a_C), C0(a_C0), theta(a_theta), useRmax(a_useRmax)
    { }
    void operator()()
    {
      // Initialize tasks
      mk_task_group;
      // Loop over child cells
      for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++) {
        // Instantiate recursive functor
        PostOrderTraversal postOrderTraversal(CC, C0, theta, useRmax);
        // Create new task for recursive call
        create_taskc(postOrderTraversal);
      }
      // Synchronize tasks
      wait_tasks;
      // Initialize multipole expansion coefficients
      C->M = 0;
      // Initialize local expansion coefficients
      C->L = 0;
      // P2M kernel
      if (C->NCHILD==0) {
        kernel::P2M(C);
      }
      // M2M kernel
      else {
        kernel::M2M(C, C0);
      }
      // Redefine cell radius R based on maximum distance
      if (useRmax) setRmax();
      // Divide R by theta
      C->R /= theta;
    }
  };

  /// Recursive functor for the pre-order traversal during downward pass
  struct PreOrderTraversal
  {
    /// Iterator of current cell
    C_iter C;
    /// Iterator of first cell
    C_iter C0;
    PreOrderTraversal(C_iter a_C, C_iter a_C0)
      : C(a_C), C0(a_C0)
    { }

    void operator()()
    {
      // L2L kernel
      kernel::L2L(C, C0);
      // L2P kernel
      if (C->NCHILD==0) kernel::L2P(C);
#if USE_WEIGHT
      // Parent cell
      C_iter CP = C0 + C->IPARENT;
      // Add parent's weight
      C->WEIGHT += CP->WEIGHT;
      // If leaf cell
      if (C->NCHILD==0) {
        // Loop over bodies in cell
        for (B_iter B=C->BODY; B!=C->BODY+C->NBODY; B++) {
          // Add cell weights to bodies
          B->WEIGHT += C->WEIGHT;
        }
      }
#endif
      mk_task_group;
      // Initialize tasks
      // Loop over child cells
      for (C_iter CC=C0+C->ICHILD; CC!=C0+C->ICHILD+C->NCHILD; CC++) {
        // Instantiate recursive functor
        PreOrderTraversal preOrderTraversal(CC, C0);
        // Create new task for recursive call
        create_taskc(preOrderTraversal);
      }
      wait_tasks;
      // Synchronize tasks
    }
  };

public:
  /// Constructor
  UpDownPass(real_t a_theta, bool a_useRmax, bool a_useRopt)
    : theta(a_theta), useRmax(a_useRmax), useRopt(a_useRopt)
  { }

  /// Upward pass (P2M, M2M)
  void upwardPass(Cells & cells)
  {
    // Start timer
    logger::startTimer("Upward pass");
    // If cell vector is not empty
    if (!cells.empty()) {
      // Set iterator of target root cell
      C_iter C0 = cells.begin();
      // Instantiate recursive functor
      PostOrderTraversal postOrderTraversal(C0, C0, theta, useRmax);
      // Recursive call for upward pass
      postOrderTraversal();
      // Root coefficient
      real_t c = (1 - theta) * (1 - theta) / std::pow(theta,P+2)
                 / powf(std::abs(C0->M[0]),1.0/3);
      // If using error optimized theta
      if (useRopt) {
        // Instantiate recursive functor
        SetRopt setRopt(C0, C0, c, theta);
        // Error optimization of R
        setRopt();
      }
    }
    // Stop timer
    logger::stopTimer("Upward pass");
  }

  /// Downward pass (L2L, L2P)
  void downwardPass(Cells & cells)
  {
    // Start timer
    logger::startTimer("Downward pass");
    // If cell vector is not empty
    if (!cells.empty()) {
      // Root cell
      C_iter C0 = cells.begin();
      // If root is the only cell do L2P
      if (C0->NCHILD == 0) kernel::L2P(C0);
      // Initialize tasks
      mk_task_group;
      // Loop over child cells
      for (C_iter CC=C0+C0->ICHILD; CC!=C0+C0->ICHILD+C0->NCHILD; CC++) {
        // Instantiate recursive functor
        PreOrderTraversal preOrderTraversal(CC, C0);
        // Recursive call for downward pass
        create_taskc(preOrderTraversal);
      }
      // Synchronize tasks
      wait_tasks;
    }
    // Stop timer
    logger::stopTimer("Downward pass");
  }

  /// Get dipole of entire system
  vec3 getDipole(Bodies & bodies, vec3 X0)
  {
    // Initialize dipole correction
    vec3 dipole = 0;
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Calcuate dipole of the whole system
      dipole += (B->X - X0) * B->SRC;
    }
    // Return dipole
    return dipole;
  }

  /// Dipole correction
  void dipoleCorrection(Bodies & bodies, vec3 dipole, int numBodies,
                        real_t cycle)
  {
    // Precalcualte constant
    real_t coef = 4 * M_PI / (3 * cycle * cycle * cycle);
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Dipole correction for potential
      B->TRG[0] -= coef * norm(dipole) / numBodies / B->SRC;
      // Loop over dimensions
      for (int d=0; d!=3; d++) {
        // Dipole correction for forces
        B->TRG[d+1] -= coef * dipole[d];
      }
    }
  }
};

#endif // STAPL_BENCHMARKS_FMM_UP_DOWN_PASS_H
