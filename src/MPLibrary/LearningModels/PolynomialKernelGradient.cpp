#include "PolynomialKernelGradient.h"


/*----------------------------- Construction ---------------------------------*/

PolynomialKernelGradient::
PolynomialKernelGradient(const SampleSet& _supports, const WeightSet& _weights,
    const double _order, const double _coeff) :
    m_supports(_supports), m_order(_order) {
  // Initialize coefficient matrix.
  size_t rows = _supports(0).size();
  size_t cols = _supports.size();
  m_coeffs = MatrixType(rows, cols);

  // Populate coefficient matrix.
  for(size_t c = 0; c < cols; ++c)
    for(size_t r = 0; r < rows; ++r)
      m_coeffs(r, c) = _supports(c)(r) * _weights(c) * _order * _coeff;
}

/*------------------------- Gradient Computation -----------------------------*/

typename PolynomialKernelGradient::SampleType
PolynomialKernelGradient::
operator()(const SampleType& _s) const {
  // Initialize the sample-dependent term x.
  size_t num = m_supports.size();
  MatrixType x = MatrixType(num, 1);

  // Populate x.
  for(size_t i = 0; i < num; ++i)
    x(i) = pow(trans(m_supports(i)) * _s, m_order - 1);

  // Multiply by the coefficient matrix to complete the computation.
  return m_coeffs * x;
}

/*----------------------------------------------------------------------------*/
