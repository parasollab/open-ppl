#ifndef LSHNF_H_
#define LSHNF_H_

#include "NeighborhoodFinderMethod.h"

////////////////////////////////////////////////////////////////////////////////
///
/// This method is the implementation of an Approximate Neigborhood Finder
/// based on the next work:
///
/// Title: Locality-sensitive Hashing Scheme Based on P-stable Distributions
/// Authors: Datar, Mayur et. al.
/// Proceedings of the Twentieth Annual Symposium on Computational Geometry '04
///
///
/// TODO
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LSHNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename GraphType::VI          VI;
    typedef typename RoadmapType::VID       VID;

    // The a vector and b scalar that define our hash functions.
    typedef std::pair<std::vector<double>,double> hash_parameters;

    // A hash family is a set of hash functions.
    typedef std::vector<hash_parameters> hash_family;

    // The hash key (or bucket) for a hash family.
    typedef std::vector<int> hash_key;

    // Maps a hash key to a set of Cfgs.
    typedef std::multimap<hash_key, CfgType> bucket_map;

    vector<hash_family> hash_families;//the a,b paramters of all hash families
    vector<bucket_map> vectorMap;

    ///@}
    ///@name Construction
    ///@{

    LSHNF(string _dmLabel = "", bool _unconnected = false, size_t _k = 5);

    LSHNF(XMLNode& _node);

    ///@}

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    virtual void Print(std::ostream& _os) const override;
    ///@}

    template<typename InputIterator, typename OutputIterator>
      OutputIterator FindNeighbors(RoadmapType* _rmp,
          InputIterator _first, InputIterator _last, bool _fromFullRoadmap,
          const CfgType& _cfg, OutputIterator _out);

    ///@}
    ///@name Auxiliar Functions
    ///@{

    //Function to generate the parameters a and b of a hash function,
    //in order to create the hash family of size k, given a r number of buckets
    std::vector<hash_parameters> HashParameters(int _r, int _k);

    //Hash function returns an individual hash of the whole hash key
    int HashFunction(const CfgType& _cfg, const hash_parameters& _h, int _r );

    //Function to compute a whole hash key
    hash_key HashFamilyFunction(const CfgType& _cfg,
        const hash_family& _h, int _r);

    //Function for getting the VIDs of the cfgs that hash to the same hash key
    //, given a query_cfg and a vector of bucketmaps
    std::vector<VID> GetNeighbors(const CfgType& _query,
          const std::vector<hash_family>& _hash_families, int _r,
          const std::vector<bucket_map>& _vectorMap);

    //Each time we add a new node to the Roadmap, we use this function for
    //mapping the new VID, by computing its hashkey and inserting it to
    //a vector of bucketmaps
    void InsertToMap(const VI _vertex, const std::vector<hash_family>& _hash_families,
    int _r, std::vector<bucket_map>& _vectorMap);

    ///@}

  private:

    double m_r;//sort of bucket radius
    int m_l;//the number of hash families
    int m_k;//the number of hash functions in a family

    double m_maxDist;//stats
    double m_minDist;//stats
    double m_avgDist;//stats
    double m_totalNodes;//stats

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LSHNF<MPTraits>::
LSHNF(string _dmLabel, bool _unconnected, size_t _k) :
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
  m_k = _node.Read("K", true, 1,0,MAX_INT, "Size of Hashkey");
  m_l = _node.Read("l", true, 1,0,MAX_INT, "Number of Hash Families");
  m_r = _node.Read("r", true, 0.0,1.0,100.0, "Bucket radius");
}


/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
LSHNF<MPTraits>::
Initialize() {

  auto env = this->GetEnvironment();
  auto boundary = env->GetBoundary();

  CfgType dummy(this->GetTask()->GetRobot());
  //Getting robot's base radius
  double radius = dummy.GetMultiBody()->GetBase()->GetBoundingSphereRadius();
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
  m_l = ceil(2 * log2(targetSize));
  m_r = ceil( pow((log10(targetSize) - 1.5),2)+1);
  m_k = 2 * dummy.PosDOF();

  //If  the base is fixed we multiply the joint angles 10x
  if(dummy.PosDOF() == 0) {
    m_k = 5;
  }
  ///////////// */

  if(this->m_debug) {
    std::cout << "Venv: " << hyper << std::endl;
    std::cout << "Vrobot: " << pow(radius,dim) << std::endl;
    std::cout << "targetSize(ratio): " << targetSize << std::endl;
    std::cout << "radius: " << radius << std::endl;
    std::cout << "k: " << m_k << " l: " << m_l << "r: " << m_r << std::endl;
  }
  // Create the hash families (l of them).
  for(int i = 0; i < m_l; ++i) {
    hash_families.push_back( HashParameters(m_r,m_k) );
  }
  //Creating empty vectorMap of size l, it will be filled later
  for(int i = 0 ; i < m_l ; ++i ) {
    // initialize bucket map l
    bucket_map map;
    vectorMap.push_back(map);
  }
  //Computing a bucket key each time we add a new vertex
  //
  auto g = this->GetRoadmap()->GetGraph();
  // Map cfgs to workspace regions whenever they are added.
  g->InstallHook(GraphType::HookType::AddVertex, this->GetNameAndLabel(),
      [this](const VI _vi){this->InsertToMap(_vi, hash_families,
        m_r, vectorMap );});
  //Initializing member varibales for debugging purposes
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
  _os << "\tk: " << this->m_k << endl;
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

  GraphType* map = _rmp->GetGraph();
  auto dmm = this->GetDistanceMetric(this->m_dmLabel);

  if(!this->m_k) {
    for(InputIterator it = _first; it != _last; ++it)
      if(map->GetVertex(it) != _cfg)
        *_out++ = make_pair(_rmp->GetGraph()->GetVID(it),
            dmm->Distance(_cfg, map->GetVertex(it)));
    return _out;
  }
  //Finding approx neighbors
  auto neighbors = GetNeighbors(_cfg, hash_families, m_r, vectorMap);

  vector<pair<VID,double>> closests;
  //Storing neighbors info (VIDs and their distances to _cfg )
  for(unsigned int i = 0 ; i < neighbors.size() ; ++i) {
    double dist = dmm->Distance(map->GetVertex(neighbors[i]), _cfg);
    closests.push_back(make_pair(neighbors[i],dist));
    //stats
    if(this->m_debug) {
      if(m_minDist > dist) {m_minDist = dist;}
      if(m_maxDist < dist) {m_maxDist = dist;}
      m_avgDist = m_avgDist + dist;
      m_totalNodes++;
      std::cout << "Max Dist: " << m_maxDist << std::endl;
      std::cout << "MIn Dist: " << m_minDist << std::endl;
      std::cout << "Avg Dist: " << m_avgDist/m_totalNodes << std::endl;
    }
  }
  if(this->m_debug) {
    std::cout << "closests.size(): " << closests.size() << std::endl;
  }
  // Returning closests
  return std::copy(closests.begin(), closests.end(), _out);

}

/*----------------------------Auxiliar functions--------------------------*/

template <typename MPTraits>
std::vector<typename std::pair<std::vector<double>,double>>
LSHNF<MPTraits>::
HashParameters(int _r, int _k) {

  double b;
  vector<double> a;
  vector<hash_parameters> family;
  CfgType dummy(this->GetTask()->GetRobot());

  for(int i = 0 ; i < _k ; ++i ){
    //b = ((double) rand() / (RAND_MAX)) * _r;
    b = DRand() * _r;
    a.clear();
    for(unsigned int j = 0 ; j < dummy.DOF() ; ++j) {
      //If  the base is fixed we multiply the joint angles 10x
      if(dummy.PosDOF() != 0 )
        a.push_back(GRand());
      else
        a.push_back(GRand()*10);
    }
    family.push_back(std::make_pair(a,b));
  }
  return family;
}

template <typename MPTraits>
int
LSHNF<MPTraits>::
HashFunction(const CfgType& _cfg, const hash_parameters& _h, int _r ) {
  const auto& _a = _h.first;
  const auto& _b = _h.second;
  double dot = std::inner_product(_a.begin(), _a.end(), _cfg.GetData().begin(),0);
  int hash = floor((dot + _b) / (double)_r);
  return hash;
}

template <typename MPTraits>
typename std::vector<int>
LSHNF<MPTraits>::
HashFamilyFunction(const CfgType& _cfg, const hash_family& _h, int _r) {
  hash_key output;
  for(const auto& parameters : _h) {
    int value = HashFunction(_cfg, parameters, _r);
    output.push_back(value);
  }
  return output;
}

template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VID>
LSHNF<MPTraits>::
GetNeighbors(const CfgType& _query, const std::vector<hash_family>& _hash_families,
      int _r, const std::vector<bucket_map>& _vectorMap) {

  auto g = this->GetRoadmap()->GetGraph();
  int count = 0;
  std::vector<VID> vectorVIDs;
  for(unsigned int i = 0 ; i < _vectorMap.size() ; ++i) {
    hash_key key = HashFamilyFunction(_query, _hash_families[i], _r);
    if (this->m_debug) {
      //for printing hash key
      std::cout << "hash_key: [";
      for(unsigned int j = 0 ; j < key.size() ; ++j) {
        std::cout << "\t" << key[j];
      }
      std::cout << "]" << std::endl;
    }
    auto range = _vectorMap[i].equal_range(key);
    for (auto it = range.first; it != range.second ; it++) {
      if (count >= this->m_k)
        return vectorVIDs;
      if(_query == it->second) // Don't connect to self
       continue;
      vectorVIDs.push_back(g->GetVID(it->second));
      ++count;
    }
  }
  return vectorVIDs;
}

template <typename MPTraits>
void
LSHNF<MPTraits>::
InsertToMap(const VI _vertex, const std::vector<hash_family>& _hash_families,
    int _r, std::vector<bucket_map>& _vectorMap) {

  const VID vid = _vertex->descriptor();
  const CfgType& _cfg = this->GetRoadmap()->GetGraph()->GetVertex(vid);
  for(unsigned int i = 0 ; i < _vectorMap.size() ; ++i ) {
    // bucket  _cfg into bucket map i
    hash_key hash = HashFamilyFunction(_cfg, _hash_families[i], _r);
    _vectorMap[i].insert(std::make_pair(hash, _cfg));
  }
}

// Define an ordering operation on hash keys so that we can use them as a key in
// a multimap.
bool
operator<(const typename std::vector<int>& _h1, const typename std::vector<int>& _h2) {
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
