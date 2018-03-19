#ifndef RADIAL_KERNEL_GRADIENT_H
#define RADIAL_KERNEL_GRADIENT_H

#include "dlib/svm.h"

#include <cstddef>


////////////////////////////////////////////////////////////////////////////////
/// A decision function gradient helper for SVM models with radial basis kernels.
///
/// Time Complexity:
///
///   For a set of n support vectors with d dimensions, construction is O(1) and
///   evaluation is O(dn).
///
/// Space Complexity:
///
///   Stores a decision function reference and a kernel object. The space
///   complexity is thus O(1).
////////////////////////////////////////////////////////////////////////////////
class RadialKernelGradient {

  ///@name Local Types
  ///@{

  typedef dlib::matrix<double, 0, 1>            SampleType;
  typedef dlib::radial_basis_kernel<SampleType> KernelType;
  typedef dlib::decision_function<KernelType>   DecisionType;

  ///@}
  ///@name Internal State
  ///@{

  const DecisionType& m_decision; ///< The decision function.
  const KernelType m_kernel;      ///< The kernel function.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a gradient helper for an SVMModel decision function.
    /// @param _decision The computed decision function.
    /// @param _gamma The kernel spread parameter.
    RadialKernelGradient(const DecisionType& _decision, const double _gamma);

    ///@}
    ///@name Gradient Computation
    ///@{

    /// Compute the gradient of the decision function at _s.
    SampleType operator()(const SampleType& _s) const;

    ///@}

};

#endif
