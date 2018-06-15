#ifndef PMPL_LSHNF_H_
#define PMPL_LSHNF_H_

#include "NeighborhoodFinderMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// This method implements a locality sensitive hashing-based approximate
/// neigborhood finder.
///
/// Reference:
/// Mayur Datar and Nicloe Immorlica and Piotr Indyk and Vahab S. Mirronki.
/// "Locality-sensitive Hashing Scheme Based on P-stable Distributions".
/// Proceedings of the Twentieth Annual Symposium on Computational Geometry,
/// 2004.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LSHNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VI          VI;
    typedef typename RoadmapType::VID       VID;

    /// The a vector and b scalar that define our hash functions.
    typedef std::pair<std::vector<double>,double> hash_parameters;

    /// A hash family is a set of hash functions.
    typedef std::vector<hash_parameters> hash_family;

    /// The hash key (or bucket) for a hash family.
    typedef std::vector<int> hash_key;

    /// Maps a hash key to a set of VIDs.
    typedef std::multimap<hash_key, VID> bucket_map;

    ///@}
    ///@name Construction
    ///@{

    LSHNF(std::string _dmLabel = "", bool _unconnected = false, size_t _k = 5);

    LSHNF(XMLNode& _node);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinder Methods

    template <typename InputIterator, typename OutputIterator>
    OutputIterator FindNeighbors(RoadmapType* _rmp,
        InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
        const CfgType& _cfg, OutputIterator _out);

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    /// Generate parameters for a hash family,
    /// @return A set of parameters that compose a hash family.
    std::vector<hash_parameters> MakeHashFamily() const;

    /// Compute a single value of a hash key for some configuration.
    /// @param _cfg The configuration.
    /// @param _h The hash function.
    /// @return The hash value of _cfg using _h.
    int HashFunction(const CfgType& _cfg, const hash_parameters& _h) const;

    /// Compute a complete hash key for a configuration.
    /// @param _cfg The configuration.
    /// @param _h The hash family to use.
    /// @return A complete hash key for _cfg using _h.
    hash_key HashFamilyFunction(const CfgType& _cfg, const hash_family& _h) const;

    /// Find the mapped VIDs with the same hash key as a given query
    /// configuration. These neighbors will be in no particular order.
    /// @param _query The query configuration.
    /// @return Up to m_k VIDs that share a hash key with _query in at least one
    ///         hash map.
    std::vector<VID> GetNeighbors(const CfgType& _query) const;

    /// Add a new node to the hash maps.
    /// @param _vi An iterator to the new node in the roadmap graph.
    void InsertIntoMap(const VI _vi);

    ///@}
    ///@name Internal State
    ///@{

    double m_hashRadius;               ///< Sort-of-bucket radius in c-space.
    size_t m_hashFamilyCount; ///< The number of hash families.
    size_t m_hashDimension;   ///< The number of hash functions in a family.

    std::vector<hash_family> m_hashFamilies; ///< The set of hash families.
    std::vector<bucket_map> m_hashMaps;      ///< The set of hash maps.

    double m_maxDist;     ///< stats
    double m_minDist;     ///< stats
    double m_avgDist;     ///< stats
    double m_totalNodes;  ///< stats

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF(std::string _dmLabel, bool _unconnected, size_t _k) :
      NeighborhoodFinderMethod<MPTraits>(_dmLabel, _unconnected) {
  this->SetName("LSHNF");
  this->m_nfType = K;
  this->m_k = _k;
}


template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF(XMLNode& _node) : NeighborhoodFinderMethod<MPTraits>(_node) {
  this->SetName("LSHNF");
  this->m_nfType = K;
  this->m_k = _node.Read("k", true, 5, 0, MAX_INT, "Number of neighbors to find");

  m_hashDimension = _node.Read("hashDimension", true, 1, 1, MAX_INT,
      "Dimension of hash space.");

  m_hashFamilyCount = _node.Read("hashFamilies", true,
      size_t(1), size_t(1), std::numeric_limits<size_t>::max(),
      "Number of hash families.");

  m_hashRadius = _node.Read("hashRadius", true, 0.0,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "Radius of hash buckets in configuration space.");
}


/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LSHNF<MPTraits>::
Initialize() {
  auto boundary = this->GetEnvironment()->GetBoundary();
  auto mb = this->GetTask()->GetRobot()->GetMultiBody();

  //Getting robot's base radius
  double radius = mb->GetBase()->GetBoundingSphereRadius();
  double hyper = 1;//to compute hypervolume of the environment
  size_t dim = boundary->GetDimension();
  //Computing hypervolume Venv
  for (size_t i = 0 ; i  < dim ; ++i ) {
    hyper = hyper * (boundary->GetRange(i).max - boundary->GetRange(i).min);
  }
  size_t targetSize = ceil(hyper / ( pow(radius,dim)) );//Ratio Venv/Vrobot

  /////////////
  /* Next values for l,k and r were obtained in practice, for now they are commented
  // they could be useful in the future
  m_hashFamilyCount = ceil(2 * log2(targetSize));
  m_hashRadius = ceil( pow((log10(targetSize) - 1.5),2)+1);
  m_hashDimesion = 2 * mb->PosDOF();

  //If  the base is fixed we multiply the joint angles 10x
  if(mb->PosDOF() == 0) {
    m_hashDimension = 5;
  }
  ///////////// */

  if(this->m_debug) {
    Print(std::cout);
    std::cout << "Venv: " << hyper << std::endl;
    std::cout << "Vrobot: " << pow(radius,dim) << std::endl;
    std::cout << "targetSize(ratio): " << targetSize << std::endl;
  }

  // Create the hash families and initialize empty maps.
  m_hashMaps.resize(m_hashFamilyCount);
  m_hashFamilies.resize(m_hashFamilyCount);
  for(auto& family : m_hashFamilies)
    family = MakeHashFamily();

  // Each time we add a new vertex, compute its mapping in each hash map.
  auto g = this->GetRoadmap()->GetGraph();
  g->InstallHook(GraphType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->InsertIntoMap(_vi);});

  // Initialize member variables for debugging purposes.
  m_maxDist = 0;
  m_minDist = 1000;
  m_avgDist = 0;
  m_totalNodes = 0;
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\thash space dimension: " << m_hashDimension
      << "\tnumber of hash families: " << m_hashFamilyCount
      << "\tbucket radius: " << m_hashRadius
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Interface--------------------*/

template <typename MPTraits>
template<typename InputIterator, typename OutputIterator>
OutputIterator
LSHNF<MPTraits>::
FindNeighbors(RoadmapType* _rmp,
    InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
    const CfgType& _cfg, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(), "LSHNF::FindNeighbors");
  this->IncrementNumQueries();

  GraphType* g = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);

  // When all neighbors are requested, simply
  if(!this->m_k) {
    for(InputIterator it = _first; it != _last; ++it)
      if(g->GetVertex(it) != _cfg)
        *_out++ = make_pair(_rmp->GetGraph()->GetVID(it),
            dmm->Distance(_cfg, g->GetVertex(it)));
    return _out;
  }

  // Find the first m_k approximate nearest-neighbors from the hash map.
  auto neighbors = GetNeighbors(_cfg);

  // Store neighbors sorted by ascending distance.
  std::set<std::pair<VID, double>, CompareSecond<VID, double>> closest;

  for(unsigned int i = 0; i < neighbors.size() ;++i) {
    const VID vid = neighbors[i];
    const double distance = dmm->Distance(g->GetVertex(vid), _cfg);
    closest.insert(std::make_pair(vid, distance));

    //stats
    if(this->m_debug) {
      m_minDist = std::min(m_minDist, distance);
      m_maxDist = std::max(m_maxDist, distance);
      m_avgDist += distance;
      ++m_totalNodes;

      std::cout << "Max Dist: "   << m_maxDist
                << "\nMin Dist: " << m_minDist
                << "\nAvg Dist: " << m_avgDist / m_totalNodes
                << std::endl;
    }
  }

  if(this->m_debug)
    std::cout << "Found " << closest.size() << " neighbors."
              << std::endl;

  // Returning closest
  return std::copy(closest.begin(), closest.end(), _out);

}

/*------------------------------ Helper Functions ----------------------------*/

template <typename MPTraits>
std::vector<typename std::pair<std::vector<double>,double>>
LSHNF<MPTraits>::
MakeHashFamily() const {
  double b;
  std::vector<double> a;
  std::vector<hash_parameters> family;

  const size_t dof = this->GetTask()->GetRobot()->GetMultiBody()->DOF();

  for(size_t i = 0 ; i < m_hashDimension ; ++i) {
    // Compute m_hashDimension gaussian-random values for the a vector.
    a.clear();
    for(size_t j = 0 ; j < dof ; ++j)
      a.push_back(GRand());

    // The b scalar is a random fraction of m_hashRadius.
    b = DRand() * m_hashRadius;

    family.push_back(std::make_pair(a,b));
  }
  return family;
}


template <typename MPTraits>
int
LSHNF<MPTraits>::
HashFunction(const CfgType& _cfg, const hash_parameters& _h) const {
  const auto& _a = _h.first;
  const auto& _b = _h.second;
  const double dot = std::inner_product(_a.begin(), _a.end(),
                                        _cfg.GetData().begin(), 0);

  return std::floor((dot + _b) / m_hashRadius);
}


template <typename MPTraits>
typename std::vector<int>
LSHNF<MPTraits>::
HashFamilyFunction(const CfgType& _cfg, const hash_family& _h) const {
  hash_key output;
  for(const auto& parameters : _h)
    output.push_back(HashFunction(_cfg, parameters));
  return output;
}


template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
LSHNF<MPTraits>::
GetNeighbors(const CfgType& _query) const {
  auto g = this->GetRoadmap()->GetGraph();
  std::vector<VID> neighbors;
  neighbors.reserve(this->m_k);

  try {
    for(size_t i = 0 ; i < m_hashMaps.size() ; ++i) {
      // Compute the hash key for _query using this hash function.
      const hash_key key = HashFamilyFunction(_query, m_hashFamilies.at(i));

      // Look for configurations in the corresponding hash map which shares this
      // key.
      auto range = m_hashMaps.at(i).equal_range(key);
      for(auto it = range.first; it != range.second; ++it) {
        // Skip connections to self.
        const Cfg& cfg = g->GetVertex(it->second);
        if(_query == cfg)
         continue;

        // Add this VID to the neighbor list.
        neighbors.push_back(it->second);
        if(neighbors.size() >= this->m_k)
          return neighbors;
      }
    }
    return neighbors;
  }
  catch(const std::runtime_error& _e) {
    // If we had an out-of-range access, re-propogate the error with WHERE info.
    throw RunTimeException(WHERE) << _e.what();
  }
}


template <typename MPTraits>
void
LSHNF<MPTraits>::
InsertIntoMap(const VI _vi) {
  MethodTimer(this->GetStatClass(), "LSHNF::InsertIntoMap");

  const VID vid = _vi->descriptor();
  const CfgType& _cfg = this->GetRoadmap()->GetGraph()->GetVertex(vid);

  // Add this cfg to each hash map.
  for(unsigned int i = 0; i < m_hashMaps.size(); ++i) {
    const hash_key hash = HashFamilyFunction(_cfg, m_hashFamilies[i]);
    m_hashMaps[i].emplace(hash, vid);
  }
}


/// Define an ordering operation on hash keys so that we can use them as a key in
/// a multimap.
bool
operator<(const std::vector<int>& _h1, const std::vector<int>& _h2) {
  const size_t numElements = std::min(_h1.size(), _h2.size());
  for(size_t i = 0; i < numElements; ++i) {
    if(_h1[i] < _h2[i])
      return true;
    else if(_h2[i] < _h1[i])
      return false;
  }
  return false;
}

#endif
