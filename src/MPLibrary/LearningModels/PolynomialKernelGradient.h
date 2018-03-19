#ifndef POLYNOMIAL_KERNEL_GRADIENT_H
#define POLYNOMIAL_KERNEL_GRADIENT_H

#include "dlib/svm.h"

#include <cstddef>


/*------------------------ Polynomial Kernel Gradient ------------------------*/

////////////////////////////////////////////////////////////////////////////////
/// A decision function gradient helper for SVM models with polynomial kernels.
///
/// The gradient computation is broken down into two parts: a constant coefficient
/// matrix and a sample-dependent term. The coefficient matrix depends only on
/// the support vectors and kernel properties: it is computed only once upon
/// construction. The sample-dependent term is also (obviously) dependent on the
/// query sample: it is recomputed for each query and then multiplied with the
/// coefficient matrix to complete the gradient computation.
///
/// Time Complexity:
///
///   For a set of n support vectors with d dimensions, both construction and
///   evaluation take O(dn) time.
///
/// Space Complexity:
///
///   Stores a copy of the n normalized support vectors (O(dn)) and the
///   coefficient matrix (O(dn)). The space complexity is thus O(dn).
////////////////////////////////////////////////////////////////////////////////
class PolynomialKernelGradient {

  ///@name Local Types
  ///@{

  typedef typename dlib::matrix<double, 0, 1>      SampleType;
  typedef typename dlib::matrix<double, 0, 0>      MatrixType;
  typedef typename dlib::matrix<double, 0, 1>      WeightSet;
  typedef typename dlib::matrix<SampleType, 0, 1>  SampleSet;

  ///@}
  ///@name Internal State
  ///@{

  const SampleSet m_supports; ///< The support vectors.
  const size_t m_order;       ///< The order of the polynomial kernel used.
  MatrixType m_coeffs;        ///< The coefficient matrix.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a gradient helper for an SVMModel decision function.
    /// @param _supports The support vectors in the decision function.
    /// @param _weights  The support vector weights.
    /// @param _order    The order of the polynomial kernel used.
    /// @param _coeff    The dot-product coefficient in the kernel.
    PolynomialKernelGradient(const SampleSet& _supports,
        const WeightSet& _weights, const double _order, const double _coeff);

    ///@}
    ///@name Gradient Computation
    ///@{

    /// Compute the gradient of the decision function at _s.
    SampleType operator()(const SampleType& _s) const;

    ///@}

};

#endif
