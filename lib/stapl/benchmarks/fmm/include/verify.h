/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_VERIFY_H
#define STAPL_BENCHMARKS_FMM_VERIFY_H

#include "logger.h"

/// Verify results
class Verify
{
public:
  /// Get sum of scalar component of a vector of target bodies
  double getSumScalar(Bodies & bodies)
  {
    // Initialize difference
    double v = 0;
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Sum of scalar component
      v += B->TRG[0] * B->SRC;
    // End loop over bodies
    }
    // Return difference
    return v;
  }

  /// Get norm of scalar component of a vector of target bodies
  double getNrmScalar(Bodies & bodies)
  {
    // Initialize norm
    double v = 0;
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Norm of scalar component
      v += B->TRG[0] * B->TRG[0];
    // End loop over bodies
    }
    // Return norm
    return v;
  }

  /// Get difference between scalar component of two vectors of target bodies
  double getDifScalar(Bodies & bodies, Bodies & bodies2)
  {
    // Initialize difference
    double v = 0;
    // Set iterator of bodies2
    B_iter B2 = bodies2.begin();
    // Loop over bodies & bodies2
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++, B2++) {
      // Difference of scalar component
      v += (B->TRG[0] - B2->TRG[0]) * (B->TRG[0] - B2->TRG[0]);
    // End loop over bodies & bodies2
    }
    // Return difference
    return v;
  }

  /// Get difference between scalar component of two vectors of target bodies
  double getRelScalar(Bodies & bodies, Bodies & bodies2)
  {
    // Initialize difference
    double v = 0;
    // Set iterator of bodies2
    B_iter B2 = bodies2.begin();
    // Loop over bodies & bodies2
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++, B2++) {
      v += ((B->TRG[0] - B2->TRG[0]) * (B->TRG[0] - B2->TRG[0]))
            // Difference of scalar component
            / (B2->TRG[0] * B2->TRG[0]);
    }
    // Return difference
    return v;
  }

  /// Get norm of scalar component of a vector of target bodies
  double getNrmVector(Bodies & bodies) {
    // Initialize norm
    double v = 0;
    // Loop over bodies
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++) {
      // Norm of vector x component
      v += B->TRG[1] * B->TRG[1]
           // Norm of vector y component
           +  B->TRG[2] * B->TRG[2]
           // Norm of vector z component
           +  B->TRG[3] * B->TRG[3];
    }
    // Return norm
    return v;
  }

  /// Get difference between scalar component of two vectors of target bodies
  double getDifVector(Bodies & bodies, Bodies & bodies2)
  {
    // Initialize difference
    double v = 0;
    // Set iterator of bodies2
    B_iter B2 = bodies2.begin();
    // Loop over bodies & bodies2
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++, B2++) {
      // Difference of vector x component
      v += (B->TRG[1] - B2->TRG[1]) * (B->TRG[1] - B2->TRG[1])
           // Difference of vector y component
           +  (B->TRG[2] - B2->TRG[2]) * (B->TRG[2] - B2->TRG[2])
           // Difference of vector z component
           +  (B->TRG[3] - B2->TRG[3]) * (B->TRG[3] - B2->TRG[3]);
    }
    // Return difference
    return v;
  }

  /// Get difference between scalar component of two vectors of target bodies
  double getRelVector(Bodies & bodies, Bodies & bodies2)
  {
    // Initialize difference
    double v = 0;
    // Set iterator of bodies2
    B_iter B2 = bodies2.begin();
    // Loop over bodies & bodies2
    for (B_iter B=bodies.begin(); B!=bodies.end(); B++, B2++) {
      // Difference of vector x component
      v += ((B->TRG[1] - B2->TRG[1]) * (B->TRG[1] - B2->TRG[1]) +
           // Difference of vector y component
           (B->TRG[2] - B2->TRG[2]) * (B->TRG[2] - B2->TRG[2]) +
           // Difference of vector z component
           (B->TRG[3] - B2->TRG[3]) * (B->TRG[3] - B2->TRG[3]))
           // Norm of vector x component
           / (B2->TRG[1] * B2->TRG[1] +
           // Norm of vector y component
           B2->TRG[2] * B2->TRG[2] +
           // Norm of vector z component
           B2->TRG[3] * B2->TRG[3]);
    }
    // Return difference
    return v;
  }

  /// Print relative L2 norm scalar error
  void print(std::string title, double v)
  {
    // If verbose flag is true
    if (logger::verbose) {
      // Set format
      std::cout << std::setw(logger::stringLength) << std::left
                // Set title
                << title << " : "
                << std::setprecision(logger::decimal) << std::scientific
                // Print potential error
                << v << std::endl;
    }
  }
};

#endif // STAPL_BENCHMARKS_FMM_VERIFY_H
