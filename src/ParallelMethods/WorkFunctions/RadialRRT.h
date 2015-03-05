//////////////////////////////////
//HEADER RadialRRT.h
/////////////////////////////////

#ifndef RADIALRRT_H_
#define RADIALRRT_H_

#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPProblem/RoadmapGraph.h"
#include "MapReduceNF.h"
#include "RadialUtils.h"

using namespace psbmp;


////TODO - Move to region class

template<class MPTraits>
class RadialRegion {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPProblemType::GraphType GraphType;
  typedef CfgType RegionType;
  typedef typename MPTraits:: WeightType WeightType;
  typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;

  protected:
  MPProblemType* m_problem;
  RegionType m_data;
  vector<RegionType> m_neighbors;
  vector<VID> m_branch;
  WeightType  m_weight;
  vector<pair<size_t, VID> > m_ccs;
  LocalGraphType m_localTree;

  public:
  RadialRegion(RegionType _data=RegionType()) : m_data(_data) { }
  RadialRegion(const RadialRegion& _other)
    : m_data(_other.m_data),m_neighbors(_other.m_neighbors),m_branch(_other.m_branch),
    m_problem(_other.m_problem), m_ccs(_other.m_ccs){ }

  //friend ostream& operator<< (ostream&, const Cfg&);
  friend ostream& operator<< (ostream& _os, const RadialRegion& _r) {
    //  _r.Print(_s);
    _os << " " << _r.m_data << " " << _r.m_weight << " " << _r.m_branch.size() << " ";
    return _os;
  }

  /* void Print(ostream &os) const  {
     os << "  "  << m_data << " ";
     }*/

  // Getters
  vector<RegionType> GetNeighbors() { return m_neighbors; }
  RegionType GetCandidate() { return m_data; }
  vector<VID> GetBranch() { return  m_branch;}
  WeightType GetWeight() { return  m_weight;}
  vector<pair<size_t, VID> > GetCCs() { return m_ccs;  }
  LocalGraphType GetLocalTree() { return m_localTree; }

  // Setters
  void SetCandidate(const RegionType& _data) { m_data= _data; }
  void SetNeighbors(const vector<RegionType>& _neighbors) { m_neighbors= _neighbors; }
  void SetBranch(const vector<VID>& _branch) {m_branch = _branch;}
  void SetWeight(const WeightType _weight) {m_weight = _weight;}
  void SetMPProblem(const MPProblemType* _problem) {m_problem = _problem;}
  void SetCCs(const vector<pair<size_t, VID> >& _ccs) { m_ccs = _ccs; }
  void SetLocalTree(const LocalGraphType _localTree) {m_localTree = _localTree;}

  void AddToBranch(const vector<VID>& _branch) {
    for(int i=0; i<_branch.size(); i++)
      m_branch.push_back(_branch[i]);
  }


  void define_type(stapl::typer& _t) {
    _t.member(m_data);
    _t.member(m_neighbors);
    _t.member(m_branch);
    //_t.member(m_problem);
    _t.member(m_ccs);
    _t.member(m_localTree);
  }

};

namespace stapl {
  template <typename Accessor, class MPTraits>
    class proxy<RadialRegion<MPTraits>, Accessor>
    : public Accessor {

      private:
        typedef typename MPTraits::CfgType CfgType;
        typedef typename MPTraits::MPProblemType MPProblemType;
        typedef typename MPProblemType::VID VID;
        typedef typename MPProblemType::GraphType GraphType;
        typedef typename MPTraits:: WeightType WeightType;
        typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;
        typedef RadialRegion<MPTraits> target_t;

        friend class proxy_core_access;

      public:
        typedef CfgType RegionType;
        explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
        proxy const& operator=(target_t const& _rhs) { Accessor::write(_rhs); return *this;}

        RegionType GetCandidate() { return Accessor::invoke(&target_t::GetCandidate); }
        vector<RegionType> GetNeighbors() { return Accessor::invoke(&target_t::GetNeighbors); }
        vector<VID> GetBranch() { return Accessor::invoke(&target_t::GetBranch); }
        vector<pair<size_t, VID> > GetCCs() { return Accessor::invoke(&target_t::GetCCs); }
        WeightType GetWeight() { return Accessor::invoke(&target_t::GetWeight); }
        LocalGraphType GetLocalTree() { return Accessor::invoke(&target_t::GetLocalTree); }

        void SetCandidate(const RegionType _data) { Accessor::invoke(&target_t::SetCandidate, _data); }
        void SetNeighbors(const vector<RegionType> _neighbors) { Accessor::invoke(&target_t::SetNeighbors, _neighbors); }
        void SetBranch(const vector<VID> _branch) { Accessor::invoke(&target_t::SetBranch, _branch); }
        void SetWeight(const WeightType _weight) { Accessor::invoke(&target_t::SetWeight, _weight); }
        void SetMPProblem(const MPProblemType* _problem) {Accessor::invoke(&target_t::SetMPProblem, _problem); }
        void SetCCs(const vector<pair<size_t, VID> >& _ccs) { Accessor::invoke(&target_t::SetCCs, _ccs) ; }
        void SetLocalTree(const LocalGraphType _localTree) {Accessor::invoke(&target_t::SetLocalTree, _localTree); }

        void AddToBranch(const vector<VID> _branch) { Accessor::invoke(&target_t::AddToBranch, _branch); }
    }; //struct proxy
}

template<class MPTraits>
class ConnectRegion {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;

    MPProblemType* m_problem;
    ConnectorPointer m_connector;

  public:
    ConnectRegion(MPProblemType* _problem, ConnectorPointer _connector) :m_problem(_problem){
      m_connector = _connector;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_connector);
    }


    template<typename vertexView, typename repeatView>
      void operator()(vertexView _view, repeatView& _gview)const {

        typedef typename vertexView::adj_edges_type ADJV;
        ADJV  edges = _view.edges();
        //SOURCE REGION
        vector<VID> sVids = _view.property().GetBranch();
        //TARGET REGION

        for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
          RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();
          vector<VID> tVids = tRegion.GetBranch();

          /// NOW CONNECT
          ColorMap colorMap;
          m_connector->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), colorMap,
              sVids.begin(),sVids.end(),
              tVids.begin(),tVids.end());

        }
      }

};



template<class MPTraits>
class RadialRegionEdge{

  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef pair<VID, double> NFType;

    MPProblemType* m_problem;
    size_t  m_k;
    string m_dmLabel;

  public:
    typedef typename stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, RadialRegion<MPTraits>, WeightType> RadialRegionGraph;
    RadialRegionEdge(MPProblemType* _problem, size_t _k, string _dmLabel):
      m_problem(_problem), m_k(_k), m_dmLabel(_dmLabel) {
      }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_k);
      _t.member(m_dmLabel);
    }

    template <typename vertexView, typename repeatView>
      void operator() (vertexView _v1, repeatView _v2) {
        ///replace with call to NF
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        Environment* env = m_problem->GetEnvironment();
        //NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder(m_nf);
        CfgType cfg = _v1.property().GetCandidate();
        VID vid = _v1.descriptor();
        vector<CfgType> neighbors;


        //NFMapFunc<MPTraits>  nfMapfnc(m_problem, cfg, m_k,true, m_dmLabel);

        // vector< NFType > nfresult = map_reduce(is_coarse_wf(nfMapfnc), NFReduceFunc<MPTraits>(), _v2);

        NFMapFunc<MPTraits> nfMap;
        vector<NFType> nfresult = nfMap.FindNeighbors(env, dmm, _v2.begin(), _v2.end(), cfg, m_k);
        //Add Edge between v and its k closest


        for(typename vector<pair<VID, double> >::iterator itr = nfresult.begin(); itr != nfresult.end(); ++itr)
        {
          // PrintValue("REDGE WF DIST : ", (*itr).second);
          typename RadialRegionGraph::edge_descriptor ed1(vid, (*itr).first);
          //typename RadialRegionGraph::edge_descriptor ed2((*itr).first, vid);
          _v2.add_edge_async(ed1);
          // _v2.add_edge_async(ed2);
          ///TODO - replace by using tuple to get id, cfg and distance
          CfgType neighbor = (*(_v2.find_vertex((*itr).first))).property().GetCandidate();
          neighbors.push_back(neighbor);
          // TODO Vizmo Debug
          //VDAddEdge(cfg,neighbor);
        }
        _v1.property().SetNeighbors(neighbors);
      }


};


template<class MPTraits>
struct RadialRegionVertex{
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    MPProblemType* m_problem;
    CfgType m_root;
    double m_radius;
    string m_dmLabel;

  public:
    RadialRegionVertex(MPProblemType* _problem, CfgType _root=CfgType(), double _radius=MAX_DBL, string _dmLabel=""):
      m_problem(_problem),m_root(_root),m_radius(_radius){
        m_dmLabel = _dmLabel;
      }

    void define_type(stapl::typer& _t){
      _t.member(m_problem);
      _t.member(m_root);
      _t.member(m_radius);
      _t.member(m_dmLabel);
    }
    /// Generate random point(ray) of m_radius lenght from the center (m_root)
    /// Add point to region graph
    template <typename View>
      void operator() (View _vw) const {
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        Environment* env = m_problem->GetEnvironment();
        CfgType point = m_root;
        point.GetRandomRay(m_radius, env, dmm);
        //point.GetRandomCfg(m_radius, m_radius);
        _vw.property().SetCandidate(point);
        //VDAddNode(point);
      }

};

template<class MPTraits>
struct RadialRegionVertex2D{
  private:

    // MPProblem* m_problem;
    typedef typename MPTraits::CfgType CfgType;
    CfgType m_basePoint;
    double m_radius;
    double m_numRegions;
  public:
    RadialRegionVertex2D(size_t _numRegions, CfgType _basePoint=CfgType(), double _radius=MAX_DBL):
      m_numRegions(_numRegions),m_basePoint(_basePoint),m_radius(_radius){
      }

    void define_type(stapl::typer& _t){
      _t.member(m_basePoint);
      _t.member(m_radius);
      _t.member(m_numRegions);
    }

    /// Add point to region graph and have them evenly distributed
    template <typename View>
      void operator() (View _vw) const {
        double alpha = (double)  2*3.141596/m_numRegions;
        vector<double> pos = m_basePoint.GetPosition();
        const double xBase = m_basePoint[0];
        const double yBase = m_basePoint[1];
        double thetaBase = atan2(yBase,xBase);
        double theta = _vw.descriptor() * alpha + thetaBase;

        CfgType regionCand;
        regionCand[0] = (m_radius * cos(theta));
        regionCand[1] = (m_radius * sin(theta));

        _vw.property().SetCandidate(regionCand);
        //VDAddNode(regionCand);

      }

};



template<class MPTraits>
struct RegionVertexByCCs{
  private:

    // MPProblem* m_problem;
    typedef typename MPTraits::CfgType CfgType;
    CfgType m_basePoint;
    double m_radius;
    double m_numRegions;
  public:
    RegionVertexByCCs(size_t _numRegions, CfgType _basePoint=CfgType(), double _radius=MAX_DBL):
      m_numRegions(_numRegions),m_basePoint(_basePoint),m_radius(_radius){
      }

    void define_type(stapl::typer& _t){
      _t.member(m_basePoint);
      _t.member(m_radius);
      _t.member(m_numRegions);
    }

    /// Add point to region graph and have them evenly distributed
    template <typename Region>
      void operator() (Region _region) const {


        double alpha = (double)  2*3.141596/m_numRegions;
        vector<double> pos = m_basePoint.GetPosition();
        const double xBase = m_basePoint[0];
        const double yBase = m_basePoint[1];
        double thetaBase = atan2(yBase,xBase);
        double theta = _region.descriptor() * alpha + thetaBase;

        CfgType regionCand;
        regionCand[0] = (m_radius * cos(theta));
        regionCand[1] = (m_radius * sin(theta));

        _region.property().SetCandidate(regionCand);
        //VDAddNode(regionCand);

      }

};



template<class MPTraits>
class BuildRadialRRT {

  protected:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename GraphType::GRAPH LocalGraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef stapl::counter<stapl::default_timer> STAPLTimer;


    MPProblemType* m_problem;
    size_t m_numNodes;
    string m_dm;
    string m_vcm;
    string m_nfm;
    double m_delta;
    double m_minDist;
    CfgType m_root;
    size_t m_numAttempts;
    bool m_strictBranching;
    double m_overlap;

  public:
    BuildRadialRRT(MPProblemType* _problem, size_t _numNodes, string _dm, string _vcm, string _nfm,
        double _delta, double _minDist, CfgType _root, size_t _numAttempts, const double _overlap, bool _strictBranching) {

      m_problem = _problem;
      m_numNodes = _numNodes;
      m_dm = _dm;
      m_delta = _delta;
      m_vcm = _vcm;
      m_nfm = _nfm;
      m_minDist = _minDist;
      m_root = _root;
      m_numAttempts = _numAttempts;
      m_strictBranching = _strictBranching;
      m_overlap = _overlap;
    }

    void define_type(stapl::typer& _t) {
      _t.member(m_numNodes);
      _t.member(m_problem);
      _t.member(m_dm);
      _t.member(m_delta);
      _t.member(m_vcm);
      _t.member(m_nfm);
      _t.member(m_minDist);
      _t.member(m_root);
      _t.member(m_numAttempts);
      _t.member(m_strictBranching);
      _t.member(m_overlap);
    }

    template<typename View>
      void operator()(View _view) const {

        CfgType regionCand = _view.property().GetCandidate();
        vector<CfgType> neighbors = _view.property().GetNeighbors();

        ///Setup global variables
        DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
        NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder(m_nfm);
        Environment* env = m_problem->GetEnvironment();
        GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
        vector<VID> branch;
        branch.clear();
        size_t ctr1=0;
        size_t maxAttempts = std::max(m_numNodes,m_numAttempts);
        branch.push_back(0);
        while(branch.size()<m_numNodes && ctr1 < maxAttempts) {

          CfgType dir, nearest, newCfg;
          VID nearestVID;

          double radius = dm->Distance(env, m_root, regionCand);
          // No neighbors means only one region?
          if(neighbors.size() == 0)
            dir.GetRandomCfg(env);
          else
            dir = SelectDirection(regionCand, neighbors, radius, m_overlap);

          /*At the first iteration, root is the only node in the tree so no need to call kclosest
            This prevent everyone banging on location 0 for root, however, if statement (branching) will be
            called m_numNodes times, let's hope branch prediction is smart enough to optimize it*/
          if(branch.size() == 0){
            nearestVID = 0;
            nearest = m_root;
          } else {
            vector<VID> kClosest;
            back_insert_iterator<vector<VID> > iterBegin(kClosest);

            //Find closest from local tree
            nfp->KClosest((m_problem->GetRoadmap()),branch.begin(),branch.end(), dir, 1, iterBegin);
            nearestVID = kClosest[0];
          }
          // TODO DEBUG: do not add to root twice for each branch, remember to delete
          if (m_strictBranching && branch.size() > 1 && nearestVID == 0) continue;

          if (nearestVID == 0) {
            nearest = m_root;
          } else {
            // Hack to make sure nearest is always local
            // nearest = (*(globalTree->find_vertex(nearestVID))).property();
            nearest =   (*(globalTree->distribution().container_manager().begin()->find_vertex(nearestVID))).property();
          }

          CDInfo cdInfo;
          int weight;
          //If expansion succeeds add to global tree and a copy in local tree

          if(RRTExpand<MPTraits>(m_problem,m_vcm, m_dm, nearest, dir, newCfg, m_delta, weight, cdInfo,
                env->GetPositionRes(), env->GetOrientationRes())
              && (dm->Distance(env, newCfg, nearest) >= m_minDist)) {

            VID newVID = globalTree->add_vertex(newCfg);
            //TODO fix weight
            pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
            //globalTree->AddEdge(nearestVID, newVID,weights);
            globalTree->add_edge_async(nearestVID, newVID,weights.first);

            // TODO Fix VIZMO DEBUG
            // if (this->m_debug) VDAddNode(newCfg);
            VDAddEdge(newCfg, nearest);
            branch.push_back(newVID);
          }
          ++ctr1;
        }
        PrintValue("Max Attempts : ", ctr1);
        PrintValue("Local Tree ", branch.size());
        // TODO necessary to set branch???
        _view.property().SetBranch(branch);
      }

};





template<class MPTraits>
class BuildRadialBlindRRT {

  protected:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::GraphType GraphType;
    //typedef typename GraphType::GRAPH LocalGraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
    typedef typename stapl::part_native_view<RoadmapViewType>::view_type NativeViewType;
    typedef stapl::counter<stapl::default_timer> STAPLTimer;
    typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;

    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;
    MPProblemType* m_problem;
    size_t m_numNodes;
    size_t m_numCCIters;
    string m_dm;
    string m_vc;
    string m_nf;
    string m_CCconnection;
    double m_delta;
    double m_minDist;
    CfgType m_root;
    size_t m_numAttempts;
    bool m_strictBranching;
    double m_overlap;
    bool m_debug;

  public:
    BuildRadialBlindRRT(MPProblemType* _problem, size_t _numNodes, string _dm, string _vc, string _nf, string _CCconnection,
        double _delta, double _minDist, CfgType _root, size_t _numAttempts, size_t _numCCIters=0, double _overlap=0, bool _strictBranching=true, bool _debug=false) {

      m_problem = _problem;
      m_numNodes = _numNodes;
      m_numCCIters = _numCCIters;
      m_dm = _dm;
      m_delta = _delta;
      m_vc = _vc;
      m_nf = _nf;
      m_CCconnection = _CCconnection;
      m_minDist = _minDist;
      m_root = _root;
      m_numAttempts = _numAttempts;
      m_strictBranching = _strictBranching;
      m_overlap = _overlap;
      m_debug = _debug;
    }

    void define_type(stapl::typer& _t) {
      _t.member(m_numNodes);
      _t.member(m_numCCIters);
      _t.member(m_problem);
      _t.member(m_dm);
      _t.member(m_delta);
      _t.member(m_vc);
      _t.member(m_nf);
      _t.member(m_CCconnection);
      _t.member(m_minDist);
      _t.member(m_root);
      _t.member(m_numAttempts);
      _t.member(m_strictBranching);
      _t.member(m_overlap);
      _t.member(m_debug);
    }

    // Run //
    template<typename View>
      void operator()(View _view) const {

        CfgType regionCand = _view.property().GetCandidate();
        vector<CfgType> neighbors = _view.property().GetNeighbors();
        LocalGraphType* localTree = new LocalGraphType();
        //LocalGraphType localTree;
        localTree->clear();
        localTree->add_vertex(0, m_root);
        ///Setup global variables
        DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
        NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder(m_nf);
        Environment* env = m_problem->GetEnvironment();
        StatClass* stats = m_problem->GetStatClass();
        GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();

        // thes are used to keep work function small
        RadialUtils<MPTraits> radialUtils (m_problem, localTree, m_dm, m_vc, m_nf, m_CCconnection, m_delta, m_minDist, m_numCCIters, m_debug);
        string callee("RadialRRT::ExpandTree");
        vector<VID> branch;

        vector<VID> invalidNodes; // instead iterating over all nodes, just keep record of the invalid ones so we dont have to hit the graph every time
        vector<VID> validNodes;
        branch.clear();
        size_t attempts=0;
        size_t maxAttempts = std::max(m_numNodes,m_numAttempts);

        vector<pair<VID, VID> > pendingEdges;
        vector<int> pendingWeights;

        while(branch.size()<m_numNodes && attempts < maxAttempts) {

          CfgType dir, nearest, newCfg;
          VID nearestVID;

          double radius = dm->Distance(env, m_root, regionCand);
          // No neighbors means only one region?
          if(neighbors.size() == 0)
            dir.GetRandomCfg(env);
          else
            dir = SelectDirection(regionCand, neighbors, radius, m_overlap);

          /*At the first iteration, root is the only node in the tree so no need to call kclosest
            This prevent everyone banging on location 0 for root, however, if statement (branching) will be
            called m_numNodes times, let's hope branch prediction is smart enough to optimize it*/
          if(branch.size() == 0){
            nearestVID = 0;
            nearest = m_root;
          } else {
            vector<VID> kClosest;
            back_insert_iterator<vector<VID> > iterBegin(kClosest);

            //Find closest from local tree
            nfp->KClosest((m_problem->GetRoadmap()),branch.begin(),branch.end(), dir, 1, iterBegin);
            nearestVID = kClosest[0];
          }
          // TODO DEBUG: do not add to root twice for each branch, remember to delete
          if (m_strictBranching && branch.size() > 0 && nearestVID == 0) continue;

          if (nearestVID == 0) {
            nearest = m_root;
          } else {
            // Hack to make sure nearest is always local
            // nearest = (*(globalTree->find_vertex(nearestVID))).property();
            nearest =   (*(globalTree->distribution().container_manager().begin()->find_vertex(nearestVID))).property();
          }

          //If expansion succeeds add to global tree and a copy in local t
          //expandTree.start();
          //int samplesMade = radialUtils.ExpandTree(nearestVID, nearest, dir, pendingEdges, pendingWeights, expansionClk, process);
          int samplesMade = radialUtils.ExpandTree(branch, nearestVID, nearest, dir, pendingEdges, pendingWeights);
          //expandTree.stop();

          attempts++;

        }
          radialUtils.RemoveInvalidNodes(branch);
          radialUtils.ConnectCCs();

        /*

          for (int i=0; i<pendingEdges.size(); i++) {
          WeightType weight ("RRTExpand", pendingWeights[i]);
          globalTree->add_edge(pendingEdges[i].first, pendingEdges[i].second, weight );
          globalTree->add_edge(pendingEdges[i].second, pendingEdges[i].first, weight );
          }

          */

        //t5.start();
        // setting CCs
        ColorMap colorMap;
        vector< pair<size_t,VID> > ccs;

        stapl::sequential::get_cc_stats(*localTree,colorMap,ccs);
        _view.property().SetCCs(ccs);
        _view.property().SetLocalTree(*localTree);
        //Why are we setting branches?
        //_view.property().SetColorMap(colorMap);
        //_view.property().SetBranch(branch);

        //t5.stop();
        // PrintValue("Setting CCs: ", t5.value() );
        //t0.stop();

        // PrintValue("Max Attempts : ", attempts);

        // PrintValue("Total: ", t0.value() );

        // TODO Cesar
        // delete localTree;

      }

};


//Looks like this class is not used so I will comment it out for now
//If it is used, then there is a problem, because the computation of local CCs is suppose to be local
//Why are we making calls to remote view/color map
//Also, if this is the only place Set/Get colormap is used then we don't need the member function m_colorMap
/*template<class MPTraits>
  class ConnectRegionCCs {
  private:
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
  typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::WeightType WeightType;
  typedef typename MPProblemType::GraphType GraphType;
  typedef typename stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > ColorMap;

  MPProblemType* m_problem;
  ConnectorPointer m_connector;
  string m_dmLabel;

  public:
  ConnectRegionCCs(MPProblemType* _problem, ConnectorPointer _connector, string _dmLabel) :m_problem(_problem){
  m_connector = _connector;
  m_dmLabel = _dmLabel;
  }

  void define_type(stapl::typer &_t){
  _t.member(m_problem);
  _t.member(m_connector);
  _t.member(m_dmLabel);
  }


  template<typename vertexView, typename repeatView>
  void operator()(vertexView _view, repeatView& _gview)const {

  typedef typename vertexView::adj_edges_type ADJV;
  ADJV  edges = _view.edges();
//SOURCE REGION
vector<VID> localVIDs = _view.property().GetBranch();

//Setup MP variables
Environment* env = m_problem->GetEnvironment();
StatClass* stats = m_problem->GetStatClass();
GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();

stringstream clockName; clockName << "Component Connection";
stats->StartClock(clockName.str());

typedef stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
RoadmapViewType roadmapView  (*globalTree);
typename RoadmapViewType::view_container_type* localTree = stapl::native_view(roadmapView)[get_location_id()].get_container();


ColorMap colorMap;
vector< pair<size_t,VID> > localCCs;
vector<VID> localCC;

for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){

RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();

stapl::sequential::get_cc_stats(localTree ,colorMap, localCCs);

int randCC = LRand() % localCCs.size();
// Fill in the local CC
stapl::sequential::get_cc(globalTree, colorMap, localCCs[randCC].second, localCC);

CfgType localccCentroid = GetCentroid(globalTree, localCC);
ColorMap remoteCmap = tRegion.GetColorMap();
vector< pair<size_t,VID> > remoteCCs = tRegion.GetCCs();
VID closestRemoteCC = GetClosestCentroid(localccCentroid, remoteCCs, remoteCmap);

// Fill in the remote CC
vector<VID> remoteCC;
stapl::sequential::get_cc(globalTree, remoteCmap, closestRemoteCC, remoteCC);

/// NOW CONNECT
ColorMap colorMap2;
m_connector->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), colorMap2,
    localCC.begin(),localCC.end(),
    remoteCC.begin(),remoteCC.end());

}
}

// returns the VID of the centroid from the vec _ccs that is closest to _centroid
VID GetClosestCentroid(CfgType _centroid, vector<pair<size_t, VID> > _ccs, ColorMap _colorMap) const {
  DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dmLabel);
  Environment* env = m_problem->GetEnvironment();
  GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();


  vector<VID> cc;

  double currMinDist = MAX_DBL;
  VID currMinVID = INVALID_VID;

  for (int i=0; i<_ccs.size(); i++) {
    // TODO Cesar fix STAPL
    stapl::sequential::get_cc(globalTree,_colorMap,_ccs[i].second,cc);
    CfgType otherCentroid = GetCentroid(globalTree,cc);
    double dist = dm->Distance(env, _centroid, otherCentroid);
    if (dist < currMinDist) {
      currMinDist = dist;
      currMinVID = _ccs[i].second;
    }

    cc.clear();
  }
  return currMinVID;
}

};*/

template<typename VID>
struct RegionCCs {
  typedef  vector<pair<size_t, VID> > result_type;
  template<typename P>
    result_type operator()(P& p) const{
      return p.GetCCs();
    }
};

template<typename MPTraits>
struct RegionLocalTree {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPTraits::WeightType WeightType;
  typedef typename MPProblemType::GraphType GraphType;

  typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> result_type;
  template<typename P>
    result_type operator()(P& p) const{
      return p.GetLocalTree();
    }
};


template<class MPTraits>
class ConnectGlobalCCs {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;
    typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
    typedef pair<size_t, VID> CCType;

    MPProblemType* m_problem;
    ConnectorPointer m_connector;
    bool m_debug;

  public:
    ConnectGlobalCCs(MPProblemType* _problem, ConnectorPointer _connector, bool _debug = false) :m_problem(_problem){
      m_connector = _connector;
      m_debug = _debug;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_connector);
      _t.member(m_debug);
    }


    template<typename vertexView, typename repeatView>
      void operator()(vertexView _view, repeatView& _gview)const {

        CfgType col;
        Environment* env = m_problem->GetEnvironment();
        StatClass* stats = m_problem->GetStatClass();
        LPOutput<MPTraits> lpOutput;
        boost::shared_ptr<RegionRRTConnect<MPTraits> > rrtConnect (boost::dynamic_pointer_cast<RegionRRTConnect<MPTraits> >(m_connector));


        DistanceMetricPointer dm = m_problem->GetDistanceMetric("");
        //edges are assumed to be directed
        typedef typename vertexView::adj_edges_type ADJV;
        ADJV  edges = _view.edges();
        size_t rgsize = _gview.size();

        //SOURCE REGION
        LocalGraphType localTree = (_view.property().GetLocalTree());
        rrtConnect->SetLocalGraph(&localTree);

        vector<CCType > localCCs;
        ColorMap colorMap;
        colorMap.reset();
        stapl::sequential::get_cc_stats(localTree, colorMap, localCCs);
        queue<VID> pendingCCs;

        //TARGET REGIONS
        if(m_debug) {
          cout << endl << "Setting LocalTree" << endl;
        }
        for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
          if((_view.descriptor()) == rgsize-1 && ((*ei).target()) == rgsize-2) continue; // remove one edge from edge list

          // fill up the queue with source CCs
          for(typename vector<pair<size_t,VID> >::iterator it = localCCs.begin(); it != localCCs.end(); it++)
            pendingCCs.push((*it).second);


          LocalGraphType remoteTree = (_gview.vp_apply(((*ei).target()), RegionLocalTree<MPTraits>()));
          rrtConnect->SetRemoteGraph(&remoteTree);
          if (m_debug) {
            cout << endl << "Setting RemoteTree" << endl;
          }
          // get target CCs
          //vector<CCType > remoteCCs = (_gview.vp_apply(((*ei).target()), RegionCCs<VID>()));
          vector<CCType > remoteCCs;

          //TODO Cesar: calculate ccs locally
          colorMap.reset();
          stapl::sequential::get_cc_stats(remoteTree, colorMap, remoteCCs);

          set<VID> unconnectedCCs;
          for(typename vector<pair<size_t,VID> >::iterator it = remoteCCs.begin(); it != remoteCCs.end(); it++)
            unconnectedCCs.insert((*it).second);

          set<VID> connectedCCs;

          /*RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();
            vector<pair<size_t,VID> > tCCs = tRegion.GetCCs();*/

          vector<VID> localCC;

          while(!pendingCCs.empty()){
            VID localCCVID = pendingCCs.front();
            pendingCCs.pop();  // really?

            colorMap.reset();
            localCC.clear();
            stapl::sequential::get_cc(localTree,colorMap, localCCVID, localCC);
            if(m_debug) {
              cout << endl << "LocalCC:" << endl;
              for(int i=0; i<localCC.size(); i++) {
                cout << localCC[i] << " ";
              }
              cout << endl;
            }
            vector<VID> remoteCC;
            for(typename set<VID>::iterator it = connectedCCs.begin(); it != connectedCCs.end(); it++) {
              VID remoteCCVID = (*it);
              // remoteCC and localCC are already connected, most likely by the root, update the sets
              if ( remoteCCVID == localCCVID ) {
                break;
              }

              colorMap.reset();
              remoteCC.clear();
              stapl::sequential::get_cc(remoteTree,colorMap, remoteCCVID, remoteCC);
              if(m_debug) {
                cout << endl << "RemoteCC:" << endl;
                for(int i=0; i<remoteCC.size(); i++) {
                  cout << remoteCC[i] << " ";
                }
                cout << endl;
                cout << "(C) Connecting: " << localCCVID << " - " << remoteCCVID << endl;
              }

              if ( rrtConnect->IsConnect(m_problem->GetRoadmap(), *stats, colorMap, localCC.begin(), localCC.end(),
                    remoteCC.begin(), remoteCC.end()) ) {
                if (m_debug) cout << "Connected" << endl;
                  break;
              }

            }
            remoteCC.clear();
            for(typename set<VID>::iterator it = unconnectedCCs.begin(); it != unconnectedCCs.end();) {

              VID remoteCCVID = (*it);
              // remoteCC and localCC are already connected, most likely by the root, update the sets

              if ( remoteCCVID == localCCVID ) {

                typename set<VID>::iterator tmp = it;
                it++;
                connectedCCs.insert(*tmp);
                unconnectedCCs.erase(tmp);
                continue;
              }

              colorMap.reset();
              remoteCC.clear();
              stapl::sequential::get_cc(remoteTree,colorMap, remoteCCVID, remoteCC);

              if(m_debug) {
                cout << endl << "RemoteCC:" << endl;
                for(int i=0; i<remoteCC.size(); i++) {
                  cout << remoteCC[i] << " ";
                }
                cout << endl;
                cout << "(UC) Connecting: " << localCCVID << " - " << remoteCCVID << endl;
              }

              if ( rrtConnect->IsConnect(m_problem->GetRoadmap(), *stats, colorMap, localCC.begin(), localCC.end(),
                    remoteCC.begin(), remoteCC.end()) ) {
                typename set<VID>::iterator tmp = it;
                it++;
                connectedCCs.insert(*tmp);
                unconnectedCCs.erase(tmp);
                if (m_debug) cout << "Connected" << endl;

              } else {
                it++;
              }
            }
          }




        }
      }

};



// t = vertex descr
template<typename T>
struct concat_vector_wf
{
  typedef vector<T> result_type;

  template<typename Ref1, typename Ref2>
    result_type operator()(Ref1 const& lhs, Ref2 const& rhs)
    {
      result_type dest(lhs);
      std::copy(rhs.begin(), rhs.end(), std::back_inserter(dest));
      return dest;
    }
};


template<typename T, typename PropertyMap>
struct is_in_cc
{
  typedef vector<T> result_type;
  size_t m_ccid;
  PropertyMap m_pmap;

  is_in_cc(size_t ccid, PropertyMap pmap)
    : m_ccid(ccid), m_pmap(pmap)
  {}

  template<typename Vref>
  result_type operator() (Vref v)
  {
    if(m_pmap.get(v).get_cc() == m_ccid)
      return result_type(1,v.descriptor());
    else
      return result_type();
    return result_type();
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_ccid);
    t.member(m_pmap);
  }
};

/*

template<class MPTraits>
class DeleteInvalidCCs {
  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename MPProblemType::LocalPlannerPointer LocalPlannerPointer;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;
    typedef typename stapl::sequential::map_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
    typedef pair<size_t, VID> CCType;

    MPProblemType* m_problem;
    bool m_debug;

  public:
    DeleteInvalidCCs(MPProblemType* _problem, bool _debug = false) :m_problem(_problem){
      m_debug = _debug;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_debug);
    }


    template<typename View>
      void operator()(View _view) const
      {

        LocalGraphType localTree = (_view.property().GetLocalTree());
        GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
        vector<CCType > ccs;
        ColorMap colorMap;
        colorMap.reset();

        stapl::sequential::get_cc_stats(localTree, colorMap, ccs);

        vector<VID> cc;
        vector<VID> toBeDeleted;

        for(int i=0; i<ccs.size(); i++) {
          if(ccs[i].second != 0 || ccs[i].first == 1) {
            colorMap.reset();
            cc.clear();
            stapl::sequential::get_cc(localTree,colorMap, ccs[i].second, cc);
            for(int j=0; j<cc.size(); j++) {
              globalTree->delete_vertex(cc[j]);
            }

          }
        }

      }
};
*/
#endif
