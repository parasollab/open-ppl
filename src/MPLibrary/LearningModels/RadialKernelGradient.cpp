#include "RadialKernelGradient.h"


/*----------------------------- Construction ---------------------------------*/

RadialKernelGradient::
RadialKernelGradient(const DecisionType& _decision, const double _gamma)
  : m_decision(_decision), m_kernel(_gamma) { }

/*------------------------- Gradient Computation -----------------------------*/

typename RadialKernelGradient::SampleType
RadialKernelGradient::
operator()(const SampleType& _s) const {
  const auto& supports = m_decision.basis_vectors;
  const auto& weights  = m_decision.alpha;

  // Get the number of support vectors (n) and sample dimension (d).
  const size_t n = supports.size(),
               d = _s.size();

  // Initialize storage for the output vector and dependent terms.
  SampleType output(d),
             kernel(n);
  dlib::matrix<SampleType, 0, 1> diff(n);

  // Compute the dependent terms.
  for(size_t i = 0; i < n; ++i) {
    // diff is a set of vectors s.t. diff(i) = supports[i] - _s.
    diff(i) = supports(i) - _s;

    // kernel is the kernel term e^(gamma * |diff(i)|^2).
    kernel(i) = m_kernel(supports(i), _s);
  }

  // Compute the output gradient.
  for(size_t j = 0; j < d; ++j) {
    output(j) = 0;
    for(size_t i = 0; i < n; ++i) {
      output(j) += weights(i) * kernel(i) * diff(i)(j);
    }
    output(j) *= -2. * m_kernel.gamma;
  }

  return output;
}

/*----------------------------------------------------------------------------*/
