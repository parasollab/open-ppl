/*
 * -- General Overview --
 *
 * The CCExpansion class provides a framework for expanding CCs by
 * applying our expansion policies to each CC separately. The user should
 * specify their node selection policy, expansion bias, and number of
 * expansion nodes per component.
 *
 * -- Summary of Methods --
 *
 * Node Selection Policies
 * 1. Random: Expand from a random set of nodes from each CC.
 * 2. Farthest: Expand from the set of nodes farthest from their CC's centroid.
 * 3. Difficult: Make use of environmental clues to expand nodes that are more
 *               likely to be in difficult areas, narrow passages for example.
 *
 * Expansion Biases
 * 1. RandomExpand: Expand in random directions.
 * 2. ExpandFrom: Expand away from the source CC's centroid.
 * 3. ExpandTo: Expand to the nearest other CC's centroid.
 * 4. MedialAxisExpand: Expand along tangents to the medial-axis.
 *
*/

#ifndef CCEXPANSION_H_
#define CCEXPANSION_H_

#include "ConnectorMethod.h"
#include "Utilities/RRTExpand.h"

//#########################//
// CCExpansion Method //
//#########################//
template <class MPTraits>
class CCExpansion: public ConnectorMethod<MPTraits> {
  public:
    ///////////////////////////////
    /* Typedefs */
    ///////////////////////////////
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename RoadmapType::GraphType GraphType;
    typedef typename vector<VID>::iterator VIDIT;

    //////////////////////////////////
    /* Constructors and Destructors */
    //////////////////////////////////
    CCExpansion(MPProblemType* _problem = NULL, string _lp = "", string _nf = "", string _vc = "");
    CCExpansion(MPProblemType* _problem, XMLNodeReader& _node);
    ~CCExpansion();

    //////////////////////////////////////
    /* Print Method */
    //////////////////////////////////////
    virtual void PrintOptions(ostream& _os) const;

    //////////////////////////////////////
    /* XML Parser */
    //////////////////////////////////////
    virtual void ParseXML(XMLNodeReader& _node);

    //////////////////////////////////////
    /* Wrapper Connect() Method */
    //////////////////////////////////////
    template <class ColorMap, class InputIterator, class OutputIterator>
      void Connect(RoadmapType* _rm, StatClass& _stats,
          ColorMap& _cmap, InputIterator _itr1First, InputIterator _itr1Last,
          InputIterator _itr2First, InputIterator _itr2Last, OutputIterator _collision);

  protected:
    /////////////////////////////////////////////////////////////////////////////
    /* Updated RDMP Methods */
    /////////////////////////////////////////////////////////////////////////////
    void UpdateRoadmap(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _bump);
    void UpdateRoadmap(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _bumpPoint, CfgType& _curr);

    /////////////////////////////////////////////////////////////////////////////
    /* Utility Methods */
    /////////////////////////////////////////////////////////////////////////////
    void TargetCCInfo(RoadmapType* _rm, vector<VID>& _curCC);
    bool IsClear(RoadmapType* _rm, CfgType& bumpPoint);
    void UnitVec(CfgType& _uA);
    template <class ColorMap>
      void PreExpansionSetup(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap, int _index, vector<VID>& _allCC);
    template <class ColorMap>
      void FindNearestCC(RoadmapType* _rm, ColorMap& _cmap, VID& _curCC, vector<VID>& _allCCs);

    /////////////////////////////////////////////////////////////////////////////
    /* Node Selection Methods */
    /////////////////////////////////////////////////////////////////////////////
    void FindDifficultNodes(RoadmapType* _rm, StatClass& _stats, vector<VID>& _cc1, int _k);
    void SelectCandidates(RoadmapType* _rm, StatClass& _stats, vector<VID>& _curCC);

    /////////////////////////////////////////////////////////////////////////////
    /* CC Expansion Method Wrapper */
    /////////////////////////////////////////////////////////////////////////////
    void Expand(RoadmapType* _rm, StatClass& _stats, int _index);

    /////////////////////////////////////////////////////////////////////////////
    /* CC Expansion Methods */
    /////////////////////////////////////////////////////////////////////////////
    void RandomExpand(RoadmapType* _rm, StatClass& _stats, int _index);
    void ExpandFrom(RoadmapType* _rm, StatClass& _stats, int _index);
    void ExpandTo(RoadmapType* _rm, StatClass& _stats, int _index);
    void MedialAxisExpand(RoadmapType* _rm, StatClass& _stats, int _index);
    void MedialRecurse(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _dir, int _count);

    /////////////////////////////////////////////////////////////////////////////
    /* Compute desired direction follow the tangent of the medial-axis */
    /////////////////////////////////////////////////////////////////////////////
    void GetMedialAxisRay(RoadmapType* _rm, CfgType& _prev, CfgType& _bumpPoint, CfgType& _curr, CfgType& _dir);

    /////////////////////////////////////////////////////////////////////////////
    /* End conditions checking method */
    /////////////////////////////////////////////////////////////////////////////
    bool WithinProximity(RoadmapType* _rm, CfgType& _bumpPoint, CfgType& _target);

    //////////
    /* Data */
    //////////

    /// CC Expansion Method Names
    enum ExpansionNames {m_RE, m_ET, m_EF, m_MAE};

    /// Node Selection Method Names
    enum NodeSelections {m_R, m_F, m_D};

    /// Medial Axis Utility
    MedialAxisUtility<MPTraits> m_medialAxisUtility;

    /// General Expansion variables
    /* Number of best candidate expansion nodes */
    int m_kNodes;
    /* Maximum number of iterations per expansion chain */
    int m_nIterations;
    /* Maximum number of failed expansions */
    int m_maxFailedExpansions;
    /* Do we always use bump nodes? */
    bool m_addIntermediate;

    /// Target CC Information (Only for Biased Expansion)
    /* Avg intra CC distance */
    double m_avgIntraDistTarget;

    /// Expansion Gain Information
    /* Minimum expansion distance per iteration */
    double m_minStepDistance;
    /* Maximum expansion distance per iteration */
    double m_maxStepDistance;

    /// Connected component information
    /* Approximate Centroid of Current CC */
    CfgType m_srcCentroid;
    /* Approximate Centroid of Target CC */
    CfgType m_goalTargetNode;

    /// Expansion Information
    /* Selected Methods */
    ExpansionNames m_expansionMethod;
    NodeSelections m_nodeSelectionOption;
    /* VC and DM Labels */
    string m_vcLabel, m_dmLabel;

    /// For keeping track of expansion chains
    /* Vector containing expansion nodes */
    vector<VID> m_expansionChains;
    /* VID for all expansion nodes */
    vector<VID> m_allExpansionNodes;

};

///////////////////////////////////////////////////////////////////////////////
//////////////////////////* Method Definitions *///////////////////////////////
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/* Constructor : Default */
///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
CCExpansion<MPTraits>::CCExpansion(MPProblemType* _problem, string _lp, string _nf, string _vc)
  : ConnectorMethod<MPTraits>(),m_medialAxisUtility() {
    this->SetName("CCExpansion");
    m_dmLabel = "";
    m_vcLabel = _vc;

    m_kNodes = 1;
    m_nIterations = 1;
    m_minStepDistance = 4.0;
    m_maxStepDistance = 100.0;

    m_expansionMethod = m_RE;
    m_nodeSelectionOption = m_R;
  }

///////////////////////////////////////////////////////////////////////////////
/* Constructor : XML Inputs */
///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
CCExpansion<MPTraits>::CCExpansion(MPProblemType* _problem, XMLNodeReader& _node)
  : ConnectorMethod<MPTraits>(_problem,_node), m_medialAxisUtility(_problem,_node){
    this->SetName("CCExpansion");
    ParseXML(_node);
  }

///////////////////////////////////////////////////////////////////////////////
/* Destructor */
///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
CCExpansion<MPTraits>::~CCExpansion(){
  /* Do Nothing */
}

///////////////////////////////////////////////////////////////////////////////
/* PrintOptions */
///////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::PrintOptions(ostream& _os) const {
  /* Call parent-class PrintOptions(..) method */
  ConnectorMethod<MPTraits>::PrintOptions(_os);

  /* Print Data Members */
  _os << "\nname = " << this->GetName();
  _os << "\nnode selection policy = " << m_nodeSelectionOption;
  _os << "\nexpansion method = " << m_expansionMethod;
  _os << "\nmax step distance = " << m_maxStepDistance;
  _os << "\nmin step distance = " << m_minStepDistance;
  _os << "\nadd intermediate node = " << (m_addIntermediate ? "True" : "False");
  _os << "\ndm label = " << m_dmLabel;
  _os << "\nvc label = " << m_vcLabel;
  _os << "\niterations = " << m_nIterations;
  _os << "\nk = " << m_kNodes << endl;
}

///////////////////////////////////////////////////////////////////////////////
/* ParseXML */
///////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::ParseXML(XMLNodeReader& _node){
  m_kNodes = _node.numberXMLParameter("kNodes",true,5,0,1000,"Max number of expansion node candidates");
  m_nIterations = _node.numberXMLParameter("nIter",true,1,0,1000,"Max number of expansion iterations");
  m_maxStepDistance = _node.numberXMLParameter("maxStepDist",true,1.0,1.0,1000.0,"Max Distance traveled per iteration");
  m_minStepDistance = _node.numberXMLParameter("minStepDist",true,0.0,0.0,1000.0,"Min Distance traveled per iteration");
  m_maxFailedExpansions = _node.numberXMLParameter("maxFails",true,10,0,1000,"Max number of failed expansions");
  m_addIntermediate = ((_node.stringXMLParameter("addIntermediate",true,"false","Always add intermediate nodes?") == "true") ? true : false);
  m_dmLabel = _node.stringXMLParameter("dmLabel",true,"euclidean","VC for RRTExpand.");
  m_vcLabel = _node.stringXMLParameter("vcLabel",true,"cd2","Distance Metric for RRTExpand.");

  string nodePolicy = _node.stringXMLParameter("nodeChoice",true,m_R,"Choice of Expansion Nodes");
  std::transform(nodePolicy.begin(), nodePolicy.end(), nodePolicy.begin(), ::toupper);
  if(nodePolicy == "RANDOM")
    m_nodeSelectionOption = m_R;
  else if(nodePolicy == "FARTHEST")
    m_nodeSelectionOption = m_F;
  else if(nodePolicy == "DIFFICULT")
    m_nodeSelectionOption = m_D;
  else{
    cout << "\nERROR in connector \"" << this->GetName() << "\""
      << " with XML value for m_nodeSelectionOption \""
      << nodePolicy << "\" undefined!" << endl
      << "We only accept \"Random\", \"Farthest\", or \"Difficult\"." << endl;
    exit(-1);
  }

  string expansionBias = _node.stringXMLParameter("expansionMethod",true,"RandomExpand","Expansion Strategy");
  std::transform(expansionBias.begin(), expansionBias.end(), expansionBias.begin(), ::toupper);
  if(expansionBias == "RANDOMEXPAND")
    m_expansionMethod = m_RE;
  else if(expansionBias == "EXPANDTO")
    m_expansionMethod = m_ET;
  else if(expansionBias == "EXPANDFROM")
    m_expansionMethod = m_EF;
  else if(expansionBias == "MEDIALAXISEXPAND"){
    m_expansionMethod = m_MAE;
    cout << "\nERROR in connector \"" << this->GetName() << "\""
      << " with XML value for m_expansionMethod \""
      << expansionBias << "\" undefined!" << endl
      << "We only accept \"RandomExpand\", \"ExpandTo\", \"ExpandFrom\", or \"MedialAxisExpand\"." << endl;
    exit(-1);
  }
}


///////////////////////////////////////////////////////////////////////////////
/* Connect Function */
///////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
template<class ColorMap, class InputIterator, class OutputIterator>
void
CCExpansion<MPTraits>::Connect(RoadmapType* _rm, StatClass& _stats,
    ColorMap& _cmap,
    InputIterator _itr1First, InputIterator _itr1Last,
    InputIterator _itr2First, InputIterator _itr2Last,
    OutputIterator _collision) {

  // Get the representative VIDs
  vector< pair<size_t,VID> > tmp;
  _cmap.reset();
  get_cc_stats(*(_rm->GetGraph()),_cmap,tmp);

  // Test to see if biased expansion is possible
  if(tmp.size() <= 1 && m_expansionMethod == m_ET){
    if(this->m_debug){
      cout << "ExpansionMethod == ExpandTo" << endl;
      cout << "However, only 1 CC exists ... Defaulting to RandomExpand." << endl;
    }
    m_expansionMethod = m_RE;
  }

  // Pull the representative VIDs from the vector of pairs in order of largest -> smallest CCs
  vector<VID> allCCs;
  sort(tmp.begin(),tmp.end());
  typedef typename vector< pair<size_t,VID> >::reverse_iterator RIT;
  for(RIT rit = tmp.rbegin(); rit != tmp.rend(); ++rit)
    allCCs.push_back(rit->second);

  // Attempt expansion on each CC
  for(size_t i = 0; i < allCCs.size(); ++i){
    PreExpansionSetup(_rm, _stats, _cmap, i, allCCs);
    for(size_t j = 0; j < m_expansionChains.size(); ++j){
      Expand(_rm, _stats, j);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
/* Preprocess Method */
///////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
template <class ColorMap>
void
CCExpansion<MPTraits>::PreExpansionSetup(RoadmapType* _rm, StatClass& _stats, ColorMap& _cmap, int _index, vector<VID>& _allCC){
  vector<VID> curCC;
  _cmap.reset();
  get_cc(*_rm->GetGraph(),_cmap,*(_allCC.begin()+_index),curCC);
  m_srcCentroid = GetCentroid(_rm->GetGraph(), curCC);
  SelectCandidates(_rm, _stats, curCC);

  if(m_expansionMethod == m_ET){
    _cmap.reset();
    FindNearestCC(_rm, _cmap, _allCC[_index], _allCC);
  }
}

///////////////////////////////////////////////////////////////////////////////
/* Expansion Method Wrapper */
///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void
CCExpansion<MPTraits>::Expand(RoadmapType* _rm, StatClass& _stats, int _index){
  if(m_expansionMethod == m_RE){
    RandomExpand(_rm,_stats,_index);
  }
  else if(m_expansionMethod == m_EF){
    ExpandFrom(_rm,_stats,_index);
  }
  else if(m_expansionMethod == m_ET){
    ExpandTo(_rm,_stats,_index);
  }
  else if(m_expansionMethod == m_MAE){
    MedialAxisExpand(_rm,_stats,_index);
  }
}

/////////////////////////////////////////////////////////////////////////////
/* Random Expansion Method */
/////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void
CCExpansion<MPTraits>::RandomExpand(RoadmapType* _rm, StatClass& _stats, int _index){
  bool good;
  int weight = 0;
  CDInfo cdInfo;
  CfgType prev, node, direction, bumpPoint;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  // Expansion node
  prev = node = _rm->GetGraph()->GetVertex(m_expansionChains[_index]);

  for(int j = 0; j < m_nIterations; ++j){
    /// PRODUCE FIRST DIRECTIONAL RAY ///////////////////////////
    // Produce a long distance random directional ray

    // Expand in a random direction
    direction.GetRandomRay(m_maxStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        prev, direction, bumpPoint, m_maxStepDistance, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    /// PRODUCE SECOND DIRECTIONAL RAY ///////////////////////////
    // Expand in a second random direction
    direction.GetRandomRay(m_minStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        bumpPoint, direction, node, m_minStepDistance, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    // Update Roadmap
    UpdateRoadmap(_rm,_stats,prev,bumpPoint,node);
    prev = node;
  }
}

/////////////////////////////////////////////////////////////////////////////
/* ExpandFrom Method */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::ExpandFrom(RoadmapType* _rm, StatClass& _stats, int _index){
  bool good;
  int weight = 0;
  CDInfo cdInfo;
  CfgType prev, node, direction, bumpPoint;
  CfgType away = m_srcCentroid;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  // Expansion node
  prev = node = _rm->GetGraph()->GetVertex(m_expansionChains[_index]);

  for(int j = 0; j < m_nIterations; ++j){
    // CC might contain 1 node, this will handle the initial expansion step
    // Else, produce ray aiming directly from centroid to expansion node
    if(away == CfgType()){
      direction.GetRandomRay(m_maxStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    }
    else {
      direction = prev - away;
      dm->ScaleCfg(m_maxStepDistance, direction);
      direction = direction + prev;
    }

    // Produce first expansion ray away from the centroid
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        prev, direction, bumpPoint, m_maxStepDistance, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    // Produce second expansion ray in a random direction
    direction.GetRandomRay(m_minStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        bumpPoint, direction, node, m_minStepDistance, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    // Update Roadmap
    UpdateRoadmap(_rm,_stats,prev,bumpPoint,node);
    prev = node;
  }
}

/////////////////////////////////////////////////////////////////////////////
/* ExpandTo Method */
/////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void
CCExpansion<MPTraits>::ExpandTo(RoadmapType* _rm, StatClass& _stats, int _index){

  bool good;
  int weight = 0;
  CDInfo cdInfo;
  CfgType prev, node, direction, bumpPoint;
  CfgType target = m_goalTargetNode;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  // Expansion node
  prev = node = _rm->GetGraph()->GetVertex(m_expansionChains[_index]);

  for(int j = 0; j < m_nIterations; ++j){
    // Produce first expansion ray to the target node
    double distanceToTarget = dm->Distance(this->GetMPProblem()->GetEnvironment(), prev, target);
    direction = target;
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        prev, direction, bumpPoint, distanceToTarget, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    // Check 1: Are we within proximity to the target location?
    if(WithinProximity(_rm, bumpPoint, target)){
      UpdateRoadmap(_rm,_stats,prev,bumpPoint);
      return;
    }

    // Produce second expansion ray in a random direction
    direction.GetRandomRay(m_minStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    good = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
        bumpPoint, direction, node, m_maxStepDistance, weight, cdInfo,
        this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
        this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
    if(!good) return;

    // Update Roadmap
    UpdateRoadmap(_rm,_stats,prev,bumpPoint,node);
    prev = node;
  }
}

/////////////////////////////////////////////////////////////////////////////
/* MedialAxisExpand KickOff Method */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::MedialAxisExpand(RoadmapType* _rm, StatClass& _stats, int _index){
  int weight = 0;
  CDInfo cdInfo;
  CfgType prev, bump1, bump2, dir1, dir2;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  /// Kick-Off Expansion history with a random ray
  prev = _rm->GetGraph()->GetVertex(m_expansionChains[_index]);
  dir1.GetRandomRay(m_maxStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
  dir2 = -dir1;

  /// Perform initial expansion
  bool expand1 = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
      prev, dir1, bump1, m_maxStepDistance, weight, cdInfo,
      this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
      this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
  bool expand2 = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
      prev, dir2, bump2, m_maxStepDistance, weight, cdInfo,
      this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
      this->GetMPProblem()->GetEnvironment()->GetOrientationRes());

  /// Expansion branch 1
  if(expand1){
    /// Record bump points
    CfgType mid1 = bump1;
    /// Push bump points to the medial axis
    m_medialAxisUtility.PushToMedialAxis(mid1,this->GetMPProblem()->GetEnvironment()->GetBoundary());
    /// Update Roadmap
    UpdateRoadmap(_rm,_stats,prev,bump1,mid1);
    /// Get approximate medial axis rays for the next iteration
    GetMedialAxisRay(_rm,prev,bump1,mid1,dir1);
    /// Recursively expand using our initial history
    MedialRecurse(_rm, _stats, mid1, dir1, 0); // Kick-Off recursive expansion
  }

  /// Expansion branch 2
  if(expand2){
    /// Record bump points
    CfgType mid2 = bump2;
    /// Push bump points to the medial axis
    m_medialAxisUtility.PushToMedialAxis(mid2,this->GetMPProblem()->GetEnvironment()->GetBoundary());
    /// Update Roadmap
    UpdateRoadmap(_rm,_stats,prev,bump2,mid2);
    /// Get approximate medial axis rays for the next iteration
    GetMedialAxisRay(_rm,prev,bump2,mid2,dir2);
    /// Recursively expand using our initial history
    MedialRecurse(_rm, _stats, mid2, dir2, 0); // Kick-Off recursive expansion
  }
}

/////////////////////////////////////////////////////////////////////////////
/* Recursive MedialAxisExpand Method */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void CCExpansion<MPTraits>::MedialRecurse(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _dir, int _count){
  /// If we have reached the max number of iterations, return.
  if(_count++ >= m_nIterations){
    return;
  }

  int weight = 0;
  CDInfo cdInfo;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  /// Compute new expansion directions
  CfgType dir1 = _dir + _prev;
  CfgType dir2 = -_dir + _prev;

  /// Perform expansion
  CfgType bump1,bump2;
  bool expand1 = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
      _prev, dir1, bump1, m_maxStepDistance, weight, cdInfo,
      this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
      this->GetMPProblem()->GetEnvironment()->GetOrientationRes());
  bool expand2 = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, m_dmLabel,
      _prev, dir2, bump2, m_maxStepDistance, weight, cdInfo,
      this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
      this->GetMPProblem()->GetEnvironment()->GetOrientationRes());

  /// Expansion branch 1
  if(expand1){
    CfgType mid1 = bump1;
    m_medialAxisUtility.PushToMedialAxis(mid1,this->GetMPProblem()->GetEnvironment()->GetBoundary());
    // Ignore the first medial expansion since we may still be near our initial expansion node.
    if(_count <= 1 || IsClear(_rm, mid1)){
      UpdateRoadmap(_rm,_stats,_prev,bump1,mid1);
      GetMedialAxisRay(_rm,_prev,bump1,mid1,dir1);
      MedialRecurse(_rm, _stats, mid1, dir1, _count);
    }
  }

  /// Expansion branch 2
  if(expand2){
    CfgType mid2 = bump2;
    m_medialAxisUtility.PushToMedialAxis(mid2,this->GetMPProblem()->GetEnvironment()->GetBoundary());
    // Ignore the first medial expansion since we may still be near our initial expansion node.
    if(_count <= 1 || IsClear(_rm, mid2)){
      UpdateRoadmap(_rm,_stats,_prev,bump2,mid2);
      GetMedialAxisRay(_rm,_prev,bump2,mid2,dir2);
      MedialRecurse(_rm, _stats, mid2, dir2, _count);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
/* Method for computing intra-CC node visibility */
///////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::FindDifficultNodes(RoadmapType* _rm, StatClass& _stats, vector<VID>& _cc1, int _k){
  vector<pair<VID,double> > nodeWeights(_cc1.size());
  typedef typename vector<pair<VID,double> >::iterator PIDIT;
  PIDIT it2 = nodeWeights.begin();
  for(VIDIT it = _cc1.begin(); it != _cc1.end(); ++it){
    CfgType& cfg = _rm->GetGraph()->GetVertex(*it);
    if(!cfg.IsStat("succConnectionAttempts") || !cfg.IsStat("totalConnectionAttempts")){
      *it2++ = pair<VID,double>(*it,0.0); // Node has never had an attempted connections
    }
    else {
      *it2++ = pair<VID,double>(*it,double(cfg.GetStat("succConnectionAttempts")/cfg.GetStat("totalConnectionAttempts")));
    }
  }
  partial_sort(nodeWeights.begin(), nodeWeights.begin()+_k, nodeWeights.end(), CompareSecond<VID,double>());
  vector<VID> difficultNodes(_k);
  for(int i = 0; i < difficultNodes.size(); ++i)
    difficultNodes[i] = nodeWeights[i].first;
  _cc1 = difficultNodes;
}

/////////////////////////////////////////////////////////////////////////////
/* Get farthest nodes from the m_srcCentroid of the CC */
/////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
template<class ColorMap>
void
CCExpansion<MPTraits>::FindNearestCC(RoadmapType* _rm, ColorMap& _cmap, VID& _curCC, vector<VID>& _allCCs){
  vector<VID> ccvids;
  pair<VID,double> nearestCC(INVALID_VID, numeric_limits<double>::max());
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  for (VIDIT it = _allCCs.begin(); it != _allCCs.end(); ++it){
    // Avoid self-check
    if(*it == _curCC){
      continue;
    }
    ccvids.clear();
    _cmap.reset();
    get_cc(*(_rm->GetGraph()), _cmap, *it, ccvids);
    CfgType otherCent = GetCentroid(_rm->GetGraph(), ccvids);
    double dist = dm->Distance(this->GetMPProblem()->GetEnvironment(), m_srcCentroid, otherCent);
    if(dist < nearestCC.second){
      nearestCC = make_pair(*it, dist);
    }
  }

  get_cc(*(_rm->GetGraph()), _cmap, nearestCC.first, ccvids);
  m_goalTargetNode = GetCentroid(_rm->GetGraph(), ccvids);
  TargetCCInfo(_rm,ccvids);
}

/////////////////////////////////////////////////////////////////////////////
/* Get farthest nodes from the m_srcCentroid of the CC */
/////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void
CCExpansion<MPTraits>::TargetCCInfo(RoadmapType* _rm, vector<VID>& _curCC){

  vector< pair<VID,double> > cc1Mod;
  m_avgIntraDistTarget = 0;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  for(VIDIT it = _curCC.begin(); it != _curCC.end(); ++it){
    CfgType cfg = _rm->GetGraph()->GetVertex(*it);
    double dist = dm->Distance(this->GetMPProblem()->GetEnvironment(),cfg,m_goalTargetNode);
    cc1Mod.push_back(make_pair(*it,dist));
    m_avgIntraDistTarget = m_avgIntraDistTarget + dist;
  }

  m_avgIntraDistTarget /= cc1Mod.size();
}

/////////////////////////////////////////////////////////////////////////////
/* Get farthest nodes from the m_srcCentroid of the CC */
/////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void
CCExpansion<MPTraits>::SelectCandidates(RoadmapType* _rm, StatClass& _stats, vector<VID>& _curCC){
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  m_expansionChains.clear();

  // Find max possible number of candidate nodes
  int k = min(_curCC.size(), m_kNodes);

  // Grab up to k random candidates
  if(m_nodeSelectionOption == m_R){
    std::random_shuffle(_curCC.begin(), _curCC.end());
    m_expansionChains.assign(_curCC.begin(),_curCC.begin()+k);
  }

  // Grab up to the k farthest candidates from the centroid
  else if(m_nodeSelectionOption == m_F){
    vector< pair<VID,double> > cc1Mod;
    for(VIDIT it = _curCC.begin(); it != _curCC.end(); ++it){
      CfgType cfg = _rm->GetGraph()->GetVertex(*it);
      double dist = dm->Distance(this->GetMPProblem()->GetEnvironment(),cfg,m_srcCentroid);
      cc1Mod.push_back(make_pair(*it,dist));
    }
    std::partial_sort(cc1Mod.begin(), cc1Mod.begin()+k, cc1Mod.end(), CompareSecondReverse<VID,double>());
    for(int i = 0; i < k; ++i){
      m_expansionChains.push_back(cc1Mod[i].first);
    }
  }

  // Grab up to the k most difficult to expand to nodes
  else if(m_nodeSelectionOption == m_D){
    FindDifficultNodes(_rm,_stats,_curCC,k);
    m_expansionChains.assign(_curCC.begin(),_curCC.begin()+k);
  }
}


/////////////////////////////////////////////////////////////////////////////
/* Produce expansion ray for MedialAxisExpand(..) */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::GetMedialAxisRay(RoadmapType* _rm, CfgType& _prev, CfgType& _bumpPoint, CfgType& _curr, CfgType& _dir){

  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  CfgType uA = _bumpPoint - _prev;
  CfgType uB = _curr - _bumpPoint;

  if(uA == CfgType() || uB == CfgType()){
    uA.GetRandomRay(m_maxStepDistance, this->GetMPProblem()->GetEnvironment(), dm);
    _dir = uA;
  }

  /// Invert rays for cross-products
  uA = -uA;
  uB = -uB;

  vector<double> uA_elems(3);
  vector<double> uB_elems(3);
  for(int i = 0; i < 3; ++i){
    if(i == 2 && _curr.PosDOF() != 3){
      uA_elems[2] = 0;
      uB_elems[2] = 0;
      break;
    }
    uA_elems[i] = uA[i];
    uB_elems[i] = uB[i];
  }

  Vector3d v(uA_elems);
  Vector3d u(uB_elems);
  Vector3d n = u%v;
  Vector3d d = u%n;

  CfgType dir;
  dir[0] = d[0];
  dir[1] = d[1];
  if(_curr.PosDOF() == 3)
    dir[2] = d[2];

  UnitVec(dir);
  dm->ScaleCfg(m_maxStepDistance, dir);
  _dir = dir;
}

/////////////////////////////////////////////////////////////////////////////
/* Check if expansion node is close to target, used by the ExpandTo Method */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
bool
CCExpansion<MPTraits>::WithinProximity(RoadmapType* _rm, CfgType& _bumpPoint, CfgType& _target){
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
  double dist = dm->Distance(this->GetMPProblem()->GetEnvironment(), _bumpPoint, _target);
  return ((dist < m_avgIntraDistTarget) ? true : false);
}

/////////////////////////////////////////////////////////////////////////////
/* Update the roadmap with the new expansion nodes, only adds one node */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::UpdateRoadmap(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _bump){
  VID node = _rm->GetGraph()->AddVertex(_bump);
  m_allExpansionNodes.push_back(node);
  LPOutput<MPTraits> lpOutput;
  _rm->GetGraph()->AddEdge(_prev,_bump,lpOutput.edge);
}

/////////////////////////////////////////////////////////////////////////////
/* Update the roadmap with the new expansion nodes, may add two nodes */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::UpdateRoadmap(RoadmapType* _rm, StatClass& _stats, CfgType& _prev, CfgType& _bump, CfgType& _target){

  LPOutput<MPTraits> lpOutput;
  bool test = false;

  // If adding intermediate, ignore lp call
  if(!m_addIntermediate){
    CfgType col;
    DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();
    test = this->GetMPProblem()->GetLocalPlanner(this->m_lpMethod)->
      IsConnected(this->GetMPProblem()->GetEnvironment(), _stats, dm, _prev, _target, col, &lpOutput,
          this->GetMPProblem()->GetEnvironment()->GetPositionRes(),
          this->GetMPProblem()->GetEnvironment()->GetOrientationRes(),
          !this->m_addAllEdges);
  }

  // If (test), ignore the bump point and connect prev to target, otherwise use bump point
  if(test){
    VID node = _rm->GetGraph()->AddVertex(_target);
    m_allExpansionNodes.push_back(node);
    _rm->GetGraph()->AddEdge(_prev,_target,lpOutput.edge);
  }
  else {
    _rm->GetGraph()->AddVertex(_bump);
    VID node = _rm->GetGraph()->AddVertex(_target);
    m_allExpansionNodes.push_back(node);
    _rm->GetGraph()->AddEdge(_prev,_bump,lpOutput.edge);
    _rm->GetGraph()->AddEdge(_bump,_target,lpOutput.edge);
  }
}

/////////////////////////////////////////////////////////////////////////////
/* Convert directional ray to a unit vector */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
void
CCExpansion<MPTraits>::UnitVec(CfgType& _v){
  // Compute the l2-norm for the vector <prev,bumpPoint>
  double mag = 0;
  for(size_t i = 0; i < _v.PosDOF(); ++i)
    mag += pow(_v[i],2);
  _v /= std::sqrt(mag);
}

/////////////////////////////////////////////////////////////////////////////
/* Check if minimal distance between medial-axis expansions is maintained */
/////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
bool
CCExpansion<MPTraits>::IsClear(RoadmapType* _rm, CfgType& bumpPoint){
  if(m_maxFailedExpansions == 0)
    return true;
  int numFails = 0;
  DistanceMetricPointer dm = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod();

  // Check to make sure that we are not expanding back into the chain
  for(VIDIT it = m_allExpansionNodes.begin(); it != m_allExpansionNodes.end(); ++it){
    CfgType bb = _rm->GetGraph()->GetVertex(*it);
    bool test1 = (bb != bumpPoint); // Do not self compare!
    bool test2 = (dm->Distance(this->GetMPProblem()->GetEnvironment(), bumpPoint, bb) < m_minStepDistance);
    if(test1 && test2 && (++numFails > m_maxFailedExpansions))
        return false;
  }
  return true;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#endif

