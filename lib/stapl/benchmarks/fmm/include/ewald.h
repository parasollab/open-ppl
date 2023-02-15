/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_EWALD_H
#define STAPL_BENCHMARKS_FMM_EWALD_H

#include "logger.h"
#include "types.h"

class Ewald
{
  /// Wave structure for Ewald summation
  struct Wave
  {
    /// 3-D wave number vector
    vec3   K;
    /// real part of wave
    real_t REAL;
    /// imaginary part of wave
    real_t IMAG;
  };
  /// Vector of Wave types
  typedef std::vector<Wave> Waves;
  /// Iterator of Wave types
  typedef Waves::iterator   W_iter;

private:
  /// Number of waves in Ewald summation
  int ksize;
  /// Scaling parameter for Ewald summation
  real_t alpha;
  /// Scaling parameter for Ewald summation
  real_t sigma;
  /// Cutoff distance
  real_t cutoff;
  /// Periodic cycle
  real_t cycle;

private:
  /// Forward DFT
  void dft(Waves & waves, Bodies & bodies) const
  {
    // Scale conversion
    real_t scale = 2 * M_PI / cycle;
    // Loop over waves
    for (W_iter W=waves.begin(); W!=waves.end(); W++) {
      //  Initialize waves
      W->REAL = W->IMAG = 0;
      //  Loop over bodies
      for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
        //   Initialize phase
        real_t th = 0;
        //   Determine phase
        for (int d=0; d<3; d++) th += W->K[d] * B->X[d] * scale;
        //   Accumulate real component
        W->REAL += B->SRC * cos(th);
        //   Accumulate imaginary component
        W->IMAG += B->SRC * sin(th);
      //  End loop over bodies
      }
    // End loop over waves
    }
  }

  /// Inverse DFT
  void idft(Waves & waves, Bodies & bodies) const
  {
    // Scale conversion
    real_t scale = 2 * M_PI / cycle;
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Initialize target values
      kvec4 TRG = kreal_t(0);
      // Loop over waves
      for (W_iter W=waves.begin(); W!=waves.end(); W++) {
        // Initialize phase
        real_t th = 0;
        // Determine phase
        for (int d=0; d<3; d++) th += W->K[d] * B->X[d] * scale;
        // Temporary value
        real_t dtmp = W->REAL * sin(th) - W->IMAG * cos(th);
        // Accumulate potential
        TRG[0]     += W->REAL * cos(th) + W->IMAG * sin(th);
        // Accumulate force
        for (int d=0; d<3; d++) TRG[d+1] -= dtmp * W->K[d];
      }
      // Scale forces
      for (int d=0; d<3; d++) TRG[d+1] *= scale;
      // Copy results to bodies
      B->TRG += TRG;
    // End loop over bodies
    }
  }

  /// Initialize wave vector
  Waves initWaves() const
  {
    // Initialize wave vector
    Waves waves;
    // kmax squared
    int kmaxsq = ksize * ksize;
    // kmax as integer
    int kmax = ksize;
    // Loop over x component
    for (int l=0; l<=kmax; l++) {
      // Determine minimum y component
      int mmin = -kmax;
      // Exception for minimum y component
      if (l==0) mmin = 0;
      // Loop over y component
      for (int m=mmin; m<=kmax; m++) {
        // termine minimum z component
        int nmin = -kmax;
        // ception for minimum z component
        if (l==0 && m==0) nmin=1;
        // op over z component
        for (int n=nmin; n<=kmax; n++) {
          // ave number squared
          real_t ksq = l * l + m * m + n * n;
          // f wave number is below kmax
          if (ksq <= kmaxsq) {
            // Initialize wave structure
            Wave wave;
            // x component of k
            wave.K[0] = l;
            // y component of k
            wave.K[1] = m;
            // z component of k
            wave.K[2] = n;
            // Initialize amplitude
            wave.REAL = wave.IMAG = 0;
            // Push wave to vector
            waves.push_back(wave);
          }
        }
      }
    }
    // Return wave vector
    return waves;
  }

  /// Ewald real part P2P kernel
  void P2P(C_iter Ci, C_iter Cj, vec3 Xperiodic) const
  {
    // Loop over target bodies
    for (B_iter Bi=Ci->BODY; Bi!=Ci->BODY+Ci->NBODY; Bi++) {
      // Loop over source bodies
      for (B_iter Bj=Cj->BODY; Bj!=Cj->BODY+Cj->NBODY; Bj++) {
        // Distance vector from source to target
        vec3 dX = Bi->X - Bj->X - Xperiodic;
        // R^2
        real_t R2 = norm(dX);
        // Exclude self interaction and cutoff
        if (0 < R2 && R2 < cutoff * cutoff) {
          //  (R * alpha)^2
          real_t R2s = R2 * alpha * alpha;
          //  R * alpha
          real_t Rs = std::sqrt(R2s);
          //  1 / (R * alpha)
          real_t invRs = 1 / Rs;
          //  1 / (R * alpha)^2
          real_t invR2s = invRs * invRs;
          //  1 / (R * alpha)^3
          real_t invR3s = invR2s * invRs;
          real_t dtmp = Bj->SRC * (M_2_SQRTPI * exp(-R2s) * invR2s +
                                   erfc(Rs) * invR3s);
          //  Scale temporary value
          dtmp *= alpha * alpha * alpha;
          //  Ewald real potential
          Bi->TRG[0] += Bj->SRC * erfc(Rs) * invRs * alpha;
          //  x component of Ewald real force
          Bi->TRG[1] -= dX[0] * dtmp;
          //  y component of Ewald real force
          Bi->TRG[2] -= dX[1] * dtmp;
          //  z component of Ewald real force
          Bi->TRG[3] -= dX[2] * dtmp;
        }
      }
    }
  }

  /// Recursive functor for traversing tree to find neighbors
  struct Neighbor
  {
    /// Ewald object
    Ewald * ewald;
    /// Iterator of current target cell
    C_iter Ci;
    /// Iterator of current source cell
    C_iter Cj;
    /// Iterator of first source cell
    C_iter C0;
    Neighbor(Ewald * a_ewald, C_iter a_Ci, C_iter a_Cj, C_iter a_C0)
      : ewald(a_ewald), Ci(a_Ci), Cj(a_Cj), C0(a_C0)
    { }

    void operator()()
    {
      //  Distance vector from source to target
      vec3 dX = Ci->X - Cj->X;
      //  Wrap around periodic domain
      wrap(dX, ewald->cycle);
      //  Coordinate offset for periodic B.C.
      vec3 Xperiodic = Ci->X - Cj->X - dX;
      //  Scalar distance
      real_t R = std::sqrt(norm(dX));
      //  If cells are close
      if (R < 3 * ewald->cutoff) {
        //   Ewald real part
        if (Cj->NCHILD == 0) ewald->P2P(Ci,Cj,Xperiodic);
        // Loop over cell's children
        for (C_iter CC=C0+Cj->ICHILD; CC!=C0+Cj->ICHILD+Cj->NCHILD; CC++) {
                //    Instantiate recursive functor
                Neighbor neighbor(ewald, Ci, CC, C0);
          //    Recursive call
          neighbor();
        //   End loop over cell's children
        }
      //  End if for far cells
      }
    // End overload operator()
    }
  };

public:
  /// Constructor
  Ewald(int a_ksize, real_t a_alpha, real_t a_sigma, real_t a_cutoff,
        real_t a_cycle)
    : ksize(a_ksize), alpha(a_alpha), sigma(a_sigma), cutoff(a_cutoff),
      cycle(a_cycle)
  { }

  /// Ewald real part
  void realPart(Cells & cells, Cells & jcells)
  {
    // Start timer
    logger::startTimer("Ewald real part");
    // Set begin iterator of source cells
    C_iter Cj = jcells.begin();
    // Initialize tasks
    mk_task_group;
    // Loop over target cells
    for (C_iter Ci=cells.begin(); Ci!=cells.end(); Ci++) {
      //  If target cell is leaf
      if (Ci->NCHILD == 0) {
        //   Instantiate recursive functor
        Neighbor neighbor(this, Ci, Cj, Cj);
        //   Create task for recursive call
        create_taskc(neighbor);
      //  End if for leaf target cell
      }
    // End loop over target cells
    }
    // Synchronize tasks
    wait_tasks;
    // Stop timer
    logger::stopTimer("Ewald real part");
  }

  /// Subtract self term
  void selfTerm(Bodies & bodies)
  {
    //  Loop over all bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      //   Self term of Ewald real part
      B->TRG[0] -= M_2_SQRTPI * B->SRC * alpha;
    //  End loop over all bodies in cell
    }
  }

  /// Ewald wave part
  void wavePart(Bodies & bodies, Bodies & jbodies)
  {
    // Start timer
    logger::startTimer("Ewald wave part");
    // Initialize wave vector
    Waves waves = initWaves();
    // Apply DFT to bodies to get waves
    dft(waves,jbodies);
    // Scale conversion
    real_t scale = 2 * M_PI / cycle;
    // First constant
    real_t coef = .5 / M_PI / M_PI / sigma / cycle;
    // Second constant
    real_t coef2 = scale * scale / (4 * alpha * alpha);
    // Loop over waves
    for (W_iter W=waves.begin(); W!=waves.end(); W++) {
      //  Wave number squared
      real_t K2 = norm(W->K);
      //  Wave factor
      real_t factor = coef * exp(-K2 * coef2) / K2;
      //  Apply wave factor to real part
      W->REAL *= factor;
      //  Apply wave factor to imaginary part
      W->IMAG *= factor;
    // End loop over waves
    }
    // Inverse DFT
    idft(waves,bodies);
    // Stop timer
    logger::stopTimer("Ewald wave part");
  }

  void print(int stringLength)
  {
    // If verbose flag is true
    if (logger::verbose) {
      // Set format
      std::cout << std::setw(stringLength) << std::fixed << std::left
                //  Print ksize
                << "ksize" << " : " << ksize << std::endl
                //  Set format
                << std::setw(stringLength)
                //  Print alpha
                << "alpha" << " : " << alpha << std::endl
                //  Set format
                << std::setw(stringLength)
                //  Print sigma
                << "sigma" << " : " << sigma << std::endl
                //  Set format
                << std::setw(stringLength)
                //  Print cutoff
                << "cutoff" << " : " << cutoff << std::endl
                //  Set format
                << std::setw(stringLength)
                //  Print cycle
                << "cycle" << " : " << cycle << std::endl;
    // End if for verbose flag
    }
  }
};

#endif // STAPL_BENCHMARKS_FMM_EWALD_H
