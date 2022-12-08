void SamplersUseCase() {
  // Get the Sampler by the XML node label.
  std::string samplerLabel = "UniformRandom";
  SamplerMethod* sampler = this->GetSampler(samplerLabel);

  // Define the number of desired samples.
  size_t numNodes = 100;

  // Define the maximum number of attempts for each sample.
  size_t maxAttempts = 3;

  // The boundary we want to sample from. We can get a boundary from our
  // environment.
  Boundary* boundary = this->GetEnvironment()->GetBoundary();

  // A vector in which all valid generated samples will be stored.
  std::vector<CfgType> validSamples;

  // The sample method samples new points within the specified boundaries. It
  // then inserts the valid configurations into the provided vector.
  sampler->Sample(numNodes, maxAttempts, boundary,
      std::back_inserter(validSamples));


  // The set of configurations to apply the sampler rule to.
  std::vector<CfgType> inputCfgs = <Set of Configurations>;

  // The vector in which all valid filtered samples will be stored.
  std::vector<CfgType> validFilteredSamples;

  // Use the Filter method to apply the Sampler method to a list of input
  // configurations. The returned valid list will be a subset of the inputs
  // containing the resulting valid configurations.
  sampler->Filter(inputCfgs.begin(), inputCfgs.end(), maxAttempts, boundary,
      validFilteredSamples);
}
