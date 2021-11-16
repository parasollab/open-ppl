void SamplersUseCase() {
    // Sampler is used to sample points within some boundary
    // based on some distribution for the given implementation
    auto s = this->GetSampler(m_sLabel);

    // The desired number of samples and total generation attempts
    // per sample are specified, as well as the sampling boundary
    size_t num, attempts;
    Boundary* bounds;
    // An iterator is provided to store all valid samples that were generated
    std::vector<CfgType> valid;

    // The Sample method samples new points within the boundaries
    // based on the derived class Sampler method
    // and inserts valid configurations into the provided vector
    s->Sample(num, attempts, bounds, std::back_inserter(valid));

    // Use the Filter method to apply the Sampler method to a 
    // list of input configurations. The returned valid list will
    // be a subset of the inputs containing the resulting valid configurations.
    std::vector<CfgType> input, valid;
    s->Filter(input.begin(), input.end(), attempts, bounds, std::back_inserter(valid));
}