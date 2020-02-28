#ifndef PMPL_NN_COMPARE_H_
#define PMPL_NN_COMPARE_H_

#include "MPStrategyMethod.h"
#include "nonstd/io.h"

#include "nlohmann/json.hpp"

#include <fstream>


////////////////////////////////////////////////////////////////////////////////
/// Compare the performance of several nearest-neighbor methods for a given
/// local planner.
///
/// This is a strategy for evaluating a NN/DM combination for a particular local
/// planner rather than building a roadmap. The constructed roadmap will contain
/// only the test vertices and no edges. An 'oracle' nearest-neighbor process
/// will determine all connectable nearest neighbors for this LP within the
/// designated k or radius value. The test NN methods will be compared to this
/// to determine how close to ideal their behavior is.
///
/// Measures time/input size and metrics on the rate of selecting and filtering
/// the oracle percentage of the oracle neighbors:
/// - Selection is the percentage of the oracle neighbors discovered. High
///   values indicate that the method is effective at finding the ideal
///   neighbors.
/// - Rejection is the fraction of returned neighbors which are connectable.
///   High rejection indicates the method is effective at excluding
///   unconnectable configurations.
/// Samples will be generated in a set number of phases with fixed total roadmap
/// size. Each set will be added to the roadmap and re-used on subsequent
/// phases, in addition to any new samples drawn to complete the needed number.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NNCompare : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;

    ///@}
    ///@name Construction
    ///@{

    NNCompare();

    NNCompare(XMLNode& _node);

    virtual ~NNCompare() = default;

    ///@}
    ///@name MPStrategyMethod Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Run() override;

    virtual void Finalize() override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Generate samples until the roadmap reaches a specified size.
    /// @param _count The target size for the roadmap.
    void GenerateSamples(const size_t _count);

    /// Test each sample in the roadmap with the oracle NF to find the oracle
    /// set for a given source.
    /// @param _source The source vertex.
    /// @param _oracles Storage buffer for the VIDs that successfully connected
    ///                 to _source.
    void FindOracleNeighbors(const VID _source, VertexSet& _oracles);

    /// Test each compare NF to evaluate its performance against the oracle.
    /// @param _oracles The oracle neighbors.
    void Test(const VID _source, const VertexSet& _oracles);

    /// Test if two vertices are connectable.
    bool Connectable(const VID _v1, const VID _v2);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_sampler;  ///< The sampler for generating test samples.
    std::string m_lpLabel;  ///< The local planner that determines oracle neighbors.
    std::string m_dmLabel;  ///< The distance metric that determines oracle neighbors.

    /// The NF that will determine the oracle candidates. Each of its selected
    /// neighbors will be tested with the local planner; those that connect will
    /// form the oracle set.
    std::unique_ptr<NeighborhoodFinderMethod<MPTraits>> m_oracleNf;

    std::vector<std::string> m_nfLabels; ///< The NF's to compare to the oracle.

    std::vector<size_t> m_sampleCounts; ///< Total roadmap size for each round.

    nlohmann::json m_output; ///< Output for the results.

    /// Avoid repeating local plans.
    typedef std::pair<VID, VID> VIDPair;
    std::unordered_map<VIDPair, bool> m_cache;

    /// Buffer to avoid constantly re-allocating huge vectors.
    std::vector<Neighbor> m_neighborBuffer;

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
NNCompare<MPTraits>::
NNCompare() : MPStrategyMethod<MPTraits>() {
  this->SetName("NNCompare");
}


template <typename MPTraits>
NNCompare<MPTraits>::
NNCompare(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("NNCompare");

  // Ensure we didn't pick up any map evaluator labels from the base class since
  // these will be ignored here.
  if(this->m_meLabels.size())
    throw ParseException(_node.Where()) << this->GetName() << " does not use "
                                        << "map evaluators.";

  m_sampler = _node.Read("samplerLabel", true, "", "The sampler for generating "
      "test configurations.");
  m_lpLabel = _node.Read("lpLabel", true, "", "The local planner that will "
      "attempt to connect selected neighbors.");
  m_dmLabel = _node.Read("dmLabel", true, "", "The distance metric for ordering "
      "oracle neighbors.");

  // Set up the oracle NF.
  {
    // Try to read a K value.
    const size_t k = _node.Read<size_t>("k", false,
        0, 1, std::numeric_limits<size_t>::max(),
        "The number of nearest oracle neighbors to find.");
    // Try to read a radius value.
    const double radius = _node.Read("radius", false, 0.,
        std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
        "The radius to use for oracle neighbors.");

    // Assert that we read something sensible for the k/radius.
    const bool readRadius = radius != 0.,
               readK      = k != 0;
    if(readRadius and readK)
      throw ParseException(_node.Where()) << "Cannot specify both a radius and "
                                          << "a K value.";
    if(!readRadius and !readK)
      throw ParseException(_node.Where()) << "Must specify either a radius or "
                                          << "a K value.";

    // Set the NF type, internal nf, and parameter function according to the
    // chosen type.
    if(readRadius) {
      m_oracleNf.reset(new RadiusNF<MPTraits>());
      m_oracleNf->GetRadius() = radius;
    }
    else {
      m_oracleNf.reset(new BruteForceNF<MPTraits>());
      m_oracleNf->GetK() = k;
    }

    m_oracleNf->SetDMLabel(this->m_dmLabel);
  }

  for(auto& child : _node)
    if(child.Name() == "NeighborhoodFinder")
      m_nfLabels.push_back(child.Read("label", true, "", "A nearest-neighbor "
          "method to compare."));

  // Read the sample counts.
  const std::string counts = _node.Read("samples", true, "", "The target size of "
      "the roadmap for each test.");
  std::istringstream iss(counts);
  size_t buffer = 0;
  while(iss >> buffer)
    m_sampleCounts.push_back(buffer);

  // Ensure that we got at least one.
  if(m_sampleCounts.empty())
    throw ParseException(_node.Where()) << "Sample counts string '" << counts
                                        << "' was empty or contained no "
                                        << "unsigned integers.";

  // Ensure that each sample count is larger than the previous.
  for(size_t i = 1; i < m_sampleCounts.size(); ++i) {
    const bool increasing = m_sampleCounts[i - 1] < m_sampleCounts[i];
    if(!increasing)
      throw ParseException(_node.Where()) << "Sample counts must be strictly "
                                          << "increasing";
  }
}

/*------------------------------ Interface -----------------------------------*/

template <typename MPTraits>
void
NNCompare<MPTraits>::
Initialize() {
  // Point the oracle NF at the right library.
  m_oracleNf->SetMPLibrary(this->GetMPLibrary());

  // Set up the output object. Example:
  // {
  //   "2": {
  //     "nf1": {
  //       "select": [.3, .2, .9, ...],
  //       "reject": [.21, .1, .8, ...],
  //       "time": 12.3
  //     },
  //     "nf2": {
  //       ...
  //     },
  //     ...
  //   },
  //   "4": {
  //     ...
  //   }
  // }

  // Set up a subobject for each sample count.
  for(const size_t count : m_sampleCounts) {
    const std::string key = std::to_string(count);
    m_output[key] = nlohmann::json::object();

    // For each NN method, add flat arrays for success rate and time.
    for(const std::string& nfLabel : m_nfLabels)
      m_output[key][nfLabel] = {
          {"select", nlohmann::json::array()},
          {"reject", nlohmann::json::array()},
          {"time", 0}
      };
  }
}


template <typename MPTraits>
void
NNCompare<MPTraits>::
Run() {
  auto stats = this->GetStatClass();
  VertexSet oracles;

  for(const size_t count : m_sampleCounts) {
    // Reset the NF clocks.
    for(const std::string& nfLabel : m_nfLabels) {
      auto nf = this->GetNeighborhoodFinder(nfLabel);
      const std::string clockName = nf->GetNameAndLabel() + "::FindNeighbors";
      stats->ClearClock(clockName);
    }

    // Generate samples.
    GenerateSamples(count);

    // Test each roadmap vertex.
    auto r = this->GetRoadmap();
    for(auto vi = r->begin(); vi != r->end(); ++vi) {
      const VID source = vi->descriptor();

      // Find the oracle neighbors.
      oracles.clear();
      FindOracleNeighbors(source, oracles);

      // Test each NF against the oracles.
      Test(source, oracles);
    }

    // Track the NN time for each method.
    for(const std::string& nfLabel : m_nfLabels) {
      auto nf = this->GetNeighborhoodFinder(nfLabel);
      const std::string clockName = nf->GetNameAndLabel() + "::FindNeighbors";
      m_output[std::to_string(count)][nfLabel]["time"] =
          stats->GetSeconds(clockName) / r->Size();
    }
  }
}


template <typename MPTraits>
void
NNCompare<MPTraits>::
Finalize() {
  // Warn if output is suppressed and ignore.
  if(!this->m_writeOutput)
    std::cerr << "\nWarning: Ignoring output disabling on strategy '"
              << this->GetNameAndLabel() << "' since it has no other purpose."
              << std::endl;

  // Print the output object to file.
  const std::string filename = this->GetBaseFilename() + ".nn-compare.json";
  std::ofstream outfile(filename);
  outfile << m_output;
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
NNCompare<MPTraits>::
GenerateSamples(const size_t _count) {
  if(this->m_debug)
    std::cout << "Sampling new nodes..." << std::endl;

  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::Sample");
  auto r = this->GetRoadmap();
  auto s = this->GetSampler(m_sampler);
  auto boundary = this->GetEnvironment()->GetBoundary();

  // Sample one at a time to spare allocating storage for collision nodes.
  std::vector<CfgType> samples;
  while(r->Size() < _count) {
    samples.clear();
    s->Sample(1, 1000, boundary, std::back_inserter(samples));
    if(samples.size())
      r->AddVertex(samples.front());
  }

  if(this->m_debug)
    std::cout << "\tRoadmap size is now " << r->Size() << "." << std::endl;
}


template <typename MPTraits>
void
NNCompare<MPTraits>::
FindOracleNeighbors(const VID _source, VertexSet& _oracles) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindOracleNeighbors");

  // If the oracle NF is k-nearest, we'll need to temporarily swap its K value.
  // This works for radius NF too because its k-value is already zero, and the
  // radius will remain as-is.
  const size_t k = m_oracleNf->GetK();
  m_oracleNf->GetK() = 0;

  // Get oracle NF and find all candidates.
  m_neighborBuffer.clear();
  auto r = this->GetRoadmap();
  m_oracleNf->FindNeighbors(r, r->GetVertex(_source),
      std::back_inserter(m_neighborBuffer));

  m_oracleNf->GetK() = k;

  // Test each candidate, add to set if test passes.
  _oracles.clear();
  for(const Neighbor& n : m_neighborBuffer) {
    if(Connectable(_source, n.target)) {
      _oracles.insert(n.target);
      if(k and _oracles.size() == k)
        break;
    }
  }
}


template <typename MPTraits>
void
NNCompare<MPTraits>::
Test(const VID _source, const VertexSet& _oracles) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::Test");

  auto r = this->GetRoadmap();
  const size_t count = r->Size();

  // For each NN method
  //   Look for neighbors of size
  //   Check how many were in the oracle neighbors
  //   ?? Of the failures, how many were connectable?
  for(const std::string& nfLabel : m_nfLabels) {
    // Find neighbors.
    auto nf = this->GetNeighborhoodFinder(nfLabel);
    m_neighborBuffer.clear();
    nf->FindNeighbors(r, r->GetVertex(_source),
        std::back_inserter(m_neighborBuffer));

    // Make sure the method has a chance to find all oracles.
    //if(m_neighborBuffer.size() < _oracles.size())
    //  throw RunTimeException(WHERE) << "NF method '" << nfLabel
    //                                << "' returned fewer than the oracle.";

    // Check how many were oracles and how many were connectable.
    size_t oracleCount = 0;
    size_t connectable = 0;
    bool foundSource = false;
    for(const Neighbor& n : m_neighborBuffer) {
      oracleCount += _oracles.count(n.target);
      if(n.target == _source) {
        foundSource = true;
        continue;
      }
      connectable += Connectable(_source, n.target);
    }

    // Compute success rate and add to the output structure.
    const double select = double(oracleCount) / _oracles.size(),
                 reject = double(connectable)
                          / (m_neighborBuffer.size() - foundSource);
    m_output[std::to_string(count)][nfLabel]["select"].push_back(select);
    m_output[std::to_string(count)][nfLabel]["reject"].push_back(reject);
  }
}


template <typename MPTraits>
bool
NNCompare<MPTraits>::
Connectable(const VID _v1, const VID _v2) {
  // Ensure we aren't checking self-connections.
  if(_v1 == _v2)
    throw RunTimeException(WHERE) << "Shouldn't be trying self-connection.";

  // Make a cache key on the VID pair. If already cached, return the answer.
  const VIDPair key = std::minmax(_v1, _v2);
  {
    auto iter = m_cache.find(key);
    const bool cached = iter != m_cache.end();
    if(cached)
      return iter->second;
  }

  // The connection isn't cached. Get the LP and attempt a connection.
  auto lp  = this->GetLocalPlanner(m_lpLabel);
  auto r   = this->GetRoadmap();
  auto env = this->GetEnvironment();

  static LPOutput<MPTraits> lpo;
  lpo.Clear();
  const CfgType& c1 = r->GetVertex(_v1);
  const CfgType& c2 = r->GetVertex(_v2);
  const bool connected = lp->IsConnected(c1, c2, &lpo, env->GetPositionRes(),
      env->GetOrientationRes());

  return m_cache[key] = connected;
}

/*----------------------------------------------------------------------------*/

#endif
