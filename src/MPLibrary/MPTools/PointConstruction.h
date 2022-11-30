#ifndef PMPL_POINT_CONSTRUCTION_H 
#define PMPL_POINT_CONSTRUCTION_H

#include <vector>
#include "Vector.h"
#include <cstdlib>

template <typename MPTraits>
class PointConstruction : public MPBaseObject<MPTraits> {
    public:
        PointConstruction();
        PointConstruction(XMLNode& _node);

        virtual void Initialize() override;


        /// Sample on the surface of a sphere. (uses PMPL-defined Vector in mathtools)
        /// Note: this does not sample INSIDE the sphere. 
        /// @param center The D-dimensional Vector at which the sphere is centered. 
        /// @param bounds The amount of distance in each dimension from the center to the surface of the sphere. 
        ///                 In other words, bounds in each dimension if sphere is centered at (0, 0, ..., 0). 
        ///                 When argument is a double, assume all bounds same distance (hypersphere with radius=bound)
        /// @param nsamples The number of samples on the sphere surface. 
        template <size_t D>
            Vector<double, D> SampleSphereSurface(
                Vector<double, D> center, Vector<double, D> bounds);

        template <size_t D>
            std::vector<Vector<double, D>> SampleSphereSurface(
                Vector<double, D> center, Vector<double, D> bounds, int nsamples); 

        template <size_t D>
            std::vector<Vector<double, D>> SampleSphereSurface(
                Vector<double, D> center, double bounds, int nsamples); 

        /// Sample in a normal distribution. 
        /// TODO: Move this somewhere else. 
        double NormalDistribution(double mean, double variance);
    
    private:
        template<size_t D>
            Vector<double, D> ElementwiseMultiply(
                Vector<double, D> a, Vector<double, D> b);
};

template <typename MPTraits>
PointConstruction<MPTraits>::
PointConstruction() {
  this->SetName("PointConstruction");
}


template <typename MPTraits>
PointConstruction<MPTraits>::
PointConstruction(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("PointConstruction");
}

template <typename MPTraits>
void
PointConstruction<MPTraits>::
Initialize() {}

template <typename MPTraits>
template <size_t D>
std::vector<Vector<double, D>> 
PointConstruction<MPTraits>::
SampleSphereSurface(Vector<double, D> center, double bounds, int nsamples) {
    double b[D];
    for (size_t i = 0; i<D; i++) { b[i] = bounds; }
    Vector<double, D> _bounds(b);
    return SampleSphereSurface(center, _bounds, nsamples);
};

template <typename MPTraits>
template <size_t D>
std::vector<Vector<double, D>> 
PointConstruction<MPTraits>::
SampleSphereSurface(Vector<double, D> center, Vector<double, D> bounds, int nsamples) {
    std::vector<Vector<double, D>> samples;
    for (int i = 0; i < nsamples; i++) {
        samples.push_back(SampleSphereSurface(center, bounds));
    }
    
    return samples;
};

template <typename MPTraits> 
template <size_t D>
Vector<double, D>
PointConstruction<MPTraits>::
SampleSphereSurface(Vector<double, D> center, Vector<double, D> bounds) {
    // This follows the Muller Method (#3 and #19):
    // http://extremelearning.com.au/how-to-generate-uniformly-random-points-on-n-spheres-and-n-balls/

    double normal_samples[D];
    for (size_t i = 0; i < D; i++) {
        normal_samples[i] = GaussianDistribution(0, 1);
    }
    Vector<double, D> samples(normal_samples);
    if (this->m_debug) {
        std::cout << "Normal distribution sample: " << normal_samples << std::endl;
    }

    // elementwiseMultiply both before and after normalization is intentional move. 
    // if one of our bounds is 0, this becomes a (n-1)-d object. 
    // And that affects our calcuations. 
    // samples = samples.elementwiseMultiply(bounds); 
    samples = samples.selfNormalize();
    samples = ElementwiseMultiply(samples, bounds);
    samples += center;
    return samples;
}

template <typename MPTraits> 
template <size_t D>
Vector<double, D>
PointConstruction<MPTraits>::
ElementwiseMultiply(Vector<double, D> a, Vector<double, D> b) {
    Vector<double, D> multiplied;
    for (size_t i = 0; i < D; i++) {
        multiplied[i] = a[i] * b[i];
    }
    return multiplied;
}

#endif