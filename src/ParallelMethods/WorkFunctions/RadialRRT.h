//////////////////////////////////
//HEADER RadialRRT.h
/////////////////////////////////

#ifndef RADIALRRT_H_
#define RADIALRRT_H_

#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPProblem/MPTraits.h"
#include "MPProblem/RoadmapGraph.h"
#include "MapReduceNF.h"

using namespace psbmp;


////TODO - Move to region class

template<class MPTraits>
class RadialRegion {
  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPProblemType::VID VID;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPProblemType::GraphType GraphType;
  typedef typename stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
  typedef CfgType RegionType;
  typedef long unsigned int WeightType;

  protected:
  MPProblemType* m_problem;
  RegionType m_data;
  vector<RegionType> m_neighbors;
  vector<VID> m_branch;
  WeightType  m_weight;
  ColorMap m_cmap;  // refreshed everytime GetCCs is called;
  vector<pair<size_t, VID> > m_ccs;

  public:
  RadialRegion(RegionType _data=RegionType()) : m_data(_data) { }
  RadialRegion(const RadialRegion& _other) 
    : m_data(_other.m_data),m_neighbors(_other.m_neighbors),m_branch(_other.m_branch), m_problem(_other.m_problem) { }

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
  ColorMap GetColorMap() { return m_cmap; } 
  vector<pair<size_t, VID> > GetCCs() { return m_ccs;  }

  // Setters
  void SetCandidate(const RegionType& _data) { m_data= _data; }
  void SetNeighbors(const vector<RegionType>& _neighbors) { m_neighbors= _neighbors; }
  void SetBranch(const vector<VID>& _branch) {m_branch = _branch;}
  void SetWeight(const WeightType _weight) {m_weight = _weight;}
  void SetMPProblem(const MPProblemType* _problem) {m_problem = _problem;}
  void SetColorMap(const ColorMap& _cmap) { m_cmap = _cmap; } 
  void SetCCs(const vector<pair<size_t, VID> >& _ccs) { m_ccs = _ccs; } 

  void AddToBranch(const vector<VID>& _branch) {
    for(int i=0; i<_branch.size(); i++) 
      m_branch.push_back(_branch[i]);
  }


  void define_type(stapl::typer& _t) { 
    _t.member(m_data);
    _t.member(m_neighbors);
    _t.member(m_branch);
    _t.member(m_problem);
    _t.member(m_cmap);
    _t.member(m_ccs);
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
        typedef typename stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
        typedef RadialRegion<MPTraits> target_t;

        friend class proxy_core_access;

      public:
        typedef CfgType RegionType;
        typedef long unsigned int WeightType;
        explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
        operator target_t() const { return Accessor::read(); }
        proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
        proxy const& operator=(target_t const& _rhs) { Accessor::write(_rhs); return *this;}

        RegionType GetCandidate() { return Accessor::invoke(&target_t::GetCandidate); }
        vector<RegionType> GetNeighbors() { return Accessor::invoke(&target_t::GetNeighbors); }
        vector<VID> GetBranch() { return Accessor::invoke(&target_t::GetBranch); }
        vector<pair<size_t, VID> > GetCCs() { return Accessor::invoke(&target_t::GetCCs); }
        WeightType GetWeight() { return Accessor::invoke(&target_t::GetWeight); }
        ColorMap GetColorMap() { return Accessor::invoke(&target_t::GetColorMap); }

        void SetCandidate(const RegionType _data) { Accessor::invoke(&target_t::SetCandidate, _data); }
        void SetNeighbors(const vector<RegionType> _neighbors) { Accessor::invoke(&target_t::SetNeighbors, _neighbors); }
        void SetBranch(const vector<VID> _branch) { Accessor::invoke(&target_t::SetBranch, _branch); }
        void SetWeight(const WeightType _weight) { Accessor::invoke(&target_t::SetWeight, _weight); }
        void SetMPProblem(const MPProblemType* _problem) {Accessor::invoke(&target_t::SetMPProblem, _problem); }
        void SetColorMap(const ColorMap& _cmap) { Accessor::invoke(&target_t::SetColorMap, _cmap); } 
        void SetCCs(const vector<pair<size_t, VID> >& _ccs) { Accessor::invoke(&target_t::SetCCs, _ccs) ; } 
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

    MPProblemType* m_problem;
    ConnectorPointer m_ncp;

  public:
    ConnectRegion(MPProblemType* _problem, ConnectorPointer _ncp) :m_problem(_problem){
      m_ncp = _ncp;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_ncp);
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
          stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > cmap;
          m_ncp->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), cmap,
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
          VDAddEdge(cfg,neighbor);
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
        VDAddNode(point);
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
        VDAddNode(regionCand);

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
        VDAddNode(regionCand);

      }

};



template<class MPTraits>
class BuildRadialBlindRRT {

  typedef typename MPTraits::MPProblemType MPProblemType;
  typedef typename MPTraits::CfgType CfgType;
  typedef typename MPProblemType::MPStrategyPointer MPStrategyPointer;

  MPStrategyPointer m_strategyMethod;

  double m_overlap;
  double m_radius;
  CfgType m_root;
  bool m_strictBranching;

  public:

  BuildRadialBlindRRT(MPStrategyPointer _mpsm, CfgType _root, double _radius, bool _strictBranching =false, double _overlap = 0 ): m_strategyMethod(_mpsm){ 

    m_radius = _radius;
    m_overlap = _overlap;
    m_root = _root;
    m_strictBranching = _strictBranching;

  }    

  void define_type(stapl::typer& _t){

    _t.member(m_strategyMethod);
    _t.member(m_radius);
    _t.member(m_overlap);
    _t.member(m_strictBranching);
    _t.member(m_root);

  }
  BuildRadialBlindRRT(const BuildRadialBlindRRT& _wf, std::size_t offset)  {}

  // Run //
  template<typename View> 
    void operator()(View _view) const {

      CfgType regionCand = _view.property().GetCandidate();
      vector<CfgType> neighbors = _view.property().GetNeighbors();
      shared_ptr<BlindRRT<MPTraits> > blindRRT = boost::dynamic_pointer_cast<BlindRRT<MPTraits> > (m_strategyMethod);
      blindRRT->InitializeParallel(m_root, regionCand, &neighbors, m_strictBranching, m_overlap, m_radius);
      blindRRT->Run();

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
    size_t m_numCCIters;
    string m_dm;
    string m_vc;
    string m_nf;
    string m_expansionType;
    string m_CCconnection;
    double m_delta;
    double m_minDist;
    CfgType m_root;
    size_t m_numAttempts;
    bool m_strictBranching;
    double m_overlap;

  public:
    BuildRadialRRT(MPProblemType* _problem, size_t _numNodes, string _dm, string _vc, string _nf, string _CCconnection, string _expansionType,
        double _delta, double _minDist, CfgType _root, size_t _numAttempts, size_t _numCCIters=0, double _overlap=0, bool _strictBranching=true) {

      m_problem = _problem;
      m_numNodes = _numNodes;
      m_numCCIters = _numCCIters;
      m_dm = _dm;
      m_delta = _delta;
      m_vc = _vc;
      m_nf = _nf;
      m_CCconnection = _CCconnection;
      m_expansionType = _expansionType;
      m_minDist = _minDist;
      m_root = _root;
      m_numAttempts = _numAttempts;
      m_strictBranching = _strictBranching;
      m_overlap = _overlap;
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
      _t.member(m_expansionType);
      _t.member(m_minDist);
      _t.member(m_root);
      _t.member(m_numAttempts);
      _t.member(m_strictBranching);
      _t.member(m_overlap);
    }

    VID AddVertex(CfgType _cfg) const {
      VID newVID = m_problem->GetRoadmap()->GetGraph()->add_vertex(_cfg);
      //  _localTree->add_vertex(newVID, _cfg); 
      // VDAddNode(_cfg);
      return newVID;
    }

    void AddEdge(VID _vid1, VID _vid2, int _weight, vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights) const { 
      if((_vid1 == 0 || _vid2 == 0) && stapl::get_location_id() != 0) { 
        _pendingEdges.push_back(make_pair(_vid1,_vid2));
        _pendingWeights.push_back(_weight);
        return;
      }

//      pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", _weight), WeightType("RRTExpand", _weight));
      WeightType weight("RRTExpand", _weight);
/// m_problem->GetRoadmap()->GetGraph()->AddEdge(_vid1, _vid2, weights);
      m_problem->GetRoadmap()->GetGraph()->add_edge(_vid1, _vid2, weight);
      m_problem->GetRoadmap()->GetGraph()->add_edge(_vid2, _vid1, weight);
      //  _localTree->add_edge(_vid1, _vid2, _weight); 
      //  _localTree->add_edge(_vid2, _vid1, _weight); 

    }

    // Run //
    template<typename View> 
      void operator()(View _view) const {

        CfgType regionCand = _view.property().GetCandidate();
        vector<CfgType> neighbors = _view.property().GetNeighbors();
        //LocalGraphType* localTree = new LocalGraphType(); 
        //localTree->add_vertex(0, m_root);
        ///Setup global variables
        STAPLTimer t0,t1,t2,t3,t4,t5, expandTree, kclosest;
        STAPLTimer expansionClk, process; 
        DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
        NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder(m_nf);
        Environment* env = m_problem->GetEnvironment();
        GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
        StatClass* stats = m_problem->GetStatClass();
        string callee("RadialRRT::ExpandTree");
        vector<VID> branch;

        vector<VID> invalidNodes; // instead iterating over all nodes, just keep record of the invalid ones so we dont have to hit the graph every time
        vector<VID> validNodes; 
        branch.clear();
        size_t attempts=0;
        size_t maxAttempts = std::max(m_numNodes,m_numAttempts);

        vector<pair<VID, VID> > pendingEdges;
        vector<int> pendingWeights;
        
        t0.start();
        t1.start();
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
            kclosest.start();
            nearest =   (*(globalTree->distribution().container_manager().begin()->find_vertex(nearestVID))).property();
            kclosest.stop();
          }

          //If expansion succeeds add to global tree and a copy in local t
          expandTree.start();
          int samplesMade = ExpandTree(branch, nearestVID, nearest, dir, pendingEdges, pendingWeights, expansionClk, process);
          expandTree.stop();

          attempts++;

        }
        t1.stop();
        if (m_expansionType != "") { 
          t2.start(); 
          RemoveInvalidNodes(branch);
          t2.stop();
          t3.start();
          ConnectCCs(); 
          t3.stop();
        }
        t4.start();
        for (int i=0; i<pendingEdges.size(); i++) {
          pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", pendingWeights[i]), WeightType("RRTExpand", pendingWeights[i]));
          globalTree->AddEdge(pendingEdges[i].first, pendingEdges[i].second, weights);
        }
        t4.stop();


        t5.start();
        // setting CCs
        stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
        vector< pair<size_t,VID> > ccs;
        typedef stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
        RoadmapViewType roadmapView  (*globalTree);
        typename RoadmapViewType::view_container_type& localTree = stapl::native_view(roadmapView)[get_location_id()].container();

        _view.property().SetCCs(ccs);
        _view.property().SetColorMap(cmap);
        _view.property().SetBranch(branch);
        
        t5.stop();
        t0.stop();
        
        PrintValue("Max Attempts : ", attempts);
        PrintValue("Local Tree ", branch.size());
        
        PrintValue("Blind Build: ", t1.value() );
        PrintValue("K Closest: ", kclosest.value() );
        PrintValue("Expand Tree: ", expandTree.value() );
        PrintValue("AddVertex Loop: ", expansionClk.value() );
        PrintValue("AddEdge: ", process.value() );
        PrintValue("Remove Invalid: ",t2.value() );
        PrintValue("Connect CCs: ", t3.value() );
        PrintValue("Pending Edges: ", t4.value() ); 
        PrintValue("Setting CCs: ", t5.value() ); 
        PrintValue("Setting CCs: ", t5.value() ); 
        PrintValue("Total: ", t0.value() ); 

      }


    int ExpandTree(vector<VID>& _currBranch, VID _nearestVID, CfgType& _nearest, CfgType& _dir, 
        vector<pair<VID, VID> >& _pendingEdges, vector<int>& _pendingWeights, STAPLTimer& expansionClk, STAPLTimer& process) const {
      GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
      Environment* env = m_problem->GetEnvironment();
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
      CfgType newCfg;
      VID newVID = INVALID_VID;
      StatClass* stats = m_problem->GetStatClass();
      string callee("RadialRRT::ExpandTree");
      CDInfo cdInfo;
      int weight;  


      if(m_expansionType == "") {
        if(RRTExpand<MPTraits>(m_problem,m_vc, m_dm, _nearest, _dir, newCfg, m_delta, weight, cdInfo, 
              env->GetPositionRes(), env->GetOrientationRes())
            && (dm->Distance(env, newCfg, _nearest) >= m_minDist)) {

          newVID = AddVertex(newCfg);

          AddEdge(_nearestVID, newVID, weight, _pendingEdges, _pendingWeights); 
          // VDAddEdge(newCfg, _nearest);

          //pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
          //globalTree->AddEdge(_nearestVID, newVID, weights);
        }

      } else {    // we are not doing a normal expansion, call BlindExpand accordingly


        ExpansionType::Expansion expansion;
        vector<pair<CfgType, int> > expansionCfgs;  // this will contain all cfgs from start to goal inclusive
        expansionCfgs.push_back(make_pair(_nearest, 0));
        expansion = BlindRRTExpand<MPTraits>(m_problem, m_vc, m_dm, m_expansionType, 
            _nearest, _dir, expansionCfgs, m_delta, cdInfo, 
            env->GetPositionRes(), env->GetOrientationRes());

        
        if (expansion == ExpansionType::NO_EXPANSION) {  
          return 0;
        }
        CfgType& newCfg = expansionCfgs.back().first; // last cfg in the returned array is delta away from nearest
        int nodesAdded = 0;
        // If good to go, add to roadmap
        if(dm->Distance(env, newCfg, _nearest) >= m_minDist && expansion != ExpansionType::OUT_OF_BOUNDARY ) {

          // Adding Nodes
          vector<VID> expansionVIDs;
          expansionVIDs.push_back(_nearestVID);

          // we already added startCfg remember?
        expansionClk.start();
          for(size_t i=1; i<expansionCfgs.size(); i++ ) {
            CfgType& cfg2 = expansionCfgs[i].first;
            VID newVID = AddVertex(expansionCfgs[i].first);
            expansionVIDs.push_back(newVID );
            _currBranch.push_back(newVID);
          }

        expansionClk.stop();
          pair<WeightType, WeightType> weights;
          // Adding Edges
          int weight;
          process.start();
          for(size_t i=1; i<expansionCfgs.size(); i++ ) {

            // For some reason, not all nodes have the VALID label
            if(!expansionCfgs[i-1].first.IsLabel("VALID")) { 
              vc->IsValid(expansionCfgs[i-1].first, env, *stats, cdInfo, &callee); 
            }
            if(!expansionCfgs[i].first.IsLabel("VALID")) { 
              vc->IsValid(expansionCfgs[i].first, env, *stats, cdInfo, &callee); 
            }

            if(expansionCfgs[i-1].first.GetLabel("VALID") &&  
                expansionCfgs[i].first.GetLabel("VALID")) {
              weight = expansionCfgs[i].second - expansionCfgs[i-1].second; // Edge weight 
              AddEdge(expansionVIDs[i-1], expansionVIDs[i], weight, _pendingEdges, _pendingWeights);
            }

            if(expansion == ExpansionType::JUMPED) // we can only add one edge, start -> middle  
              break;
          }
          process.stop();

          nodesAdded=  expansionCfgs.size() - 1;    // substract one cause the start already belonged to the tree
        } else {  // did not reach minDist :(
          return 0;
        }

        return nodesAdded;
      }   // End Blind

      return 0; // return 
    }

    void RemoveInvalidNodes(vector<VID>& _allVIDs) const {

      RoadmapType* rdmp = m_problem->GetRoadmap();
      GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
      StatClass* stats = m_problem->GetStatClass();
      ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
      Environment* env = m_problem->GetEnvironment();
      CDInfo  cdInfo;
      string callee("BlindRRT::RemoveInvalidNodes");

      for (size_t i=0; i<_allVIDs.size(); i++) {
        VID vid = _allVIDs[i];
        CfgType cfg = (*(globalTree->distribution().container_manager().begin()->find_vertex(vid))).property();

        if (!cfg.IsLabel("VALID")) 
          vc->IsValid(cfg, env, *stats, cdInfo, &callee); 

        if (!cfg.GetLabel("VALID") ) {
          rdmp->GetGraph()->delete_vertex(_allVIDs[i]); 
          //VDRemoveNode(cfg); 
        }
      }


    }

    void ConnectCCs() const {

      //Setup MP variables
      StatClass* stats = m_problem->GetStatClass();
      RoadmapType* rdmp = m_problem->GetRoadmap();
      GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
      ValidityCheckerPointer vc = m_problem->GetValidityChecker(m_vc);
      Environment* env = m_problem->GetEnvironment();
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
      NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);
      ConnectorPointer pConnection = m_problem->GetConnector("RRTConnect");
      CDInfo  cdInfo;
      string callee("BlindRRT::RemoveInvalidNodes");

      stringstream clockName; clockName << "Component Connection";
      stats->StartClock(clockName.str());

      stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
      vector< pair<size_t,VID> > ccs;
      typedef stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
      RoadmapViewType roadmapView  (*globalTree);
      typename RoadmapViewType::view_container_type& localTree = stapl::native_view(roadmapView)[get_location_id()].container();

      stapl::sequential::get_cc_stats(localTree ,cmap, ccs);      

      cmap.reset();
      if(ccs.size()==1) return;

      vector<VID> cc1;
      vector<VID> cc2;
      VID cc1VID; 
      VID cc2VID; 

      bool alt = false;
      bool mapPassedEvaluation = false;
      size_t iters = 0;
      while(ccs.size() > 1 && iters <= m_numCCIters) {
        int rand1 = LRand() % ccs.size();
        // always expand from the root CC
        cc1VID = ccs[ rand1 ].second; 
        iters++;
        stapl::sequential::get_cc(localTree,cmap,cc1VID,cc1);
        cmap.reset();
        if (cc1.size() == 1) {

          CfgType cfg = m_problem->GetRoadmap()->GetGraph()->GetCfg(cc1[0]);

          if (!cfg.IsLabel("VALID")) 
            vc->IsValid(cfg, env, *stats, cdInfo, &callee); 
          if (!cfg.GetLabel("VALID"))
            continue;
        }
        if (m_CCconnection == "Random") {

          int rand2 = LRand() % ccs.size();
          if (rand1 == rand2) continue;
          cc2VID = ccs[ rand2 ].second; 

        } else if(m_CCconnection == "ClosestNode") {
          VID randomNode = cc1[LRand() % cc1.size()];
          cc2VID = GetClosestCC(randomNode, cc1VID);

        } else if (m_CCconnection == "ClosestCC") {
          // To be implemented 
        } else if(m_CCconnection == "Mixed") {
          if(alt) {
            int rand2 = LRand() % ccs.size();
            if (rand1 == rand2) continue;
            cc2VID = ccs[ rand2 ].second; 

          } else {
            VID randomNode = cc1[LRand() % cc1.size()];
            cc2VID = GetClosestCC(randomNode, cc1VID);
          }
          alt = !alt;

        } else {
          cout << "Unknown CC connection type: " << m_CCconnection << endl;
          exit(-1);

        }
        stapl::sequential::get_cc(localTree,cmap,cc2VID,cc2);
        cmap.reset();

        // Maybe this is an invalid node, don't use it
        if (cc2.size() == 1) {

          CfgType cfg = m_problem->GetRoadmap()->GetGraph()->GetCfg(cc2[0]);
          if (!cfg.IsLabel("VALID")) 
            vc->IsValid(cfg, env, *stats, cdInfo, &callee); 
          if (!cfg.GetLabel("VALID") )
            continue;

        }
        // We got a pair of CCs, attempt to Connect them!
        pConnection->Connect(rdmp, *stats, cmap, cc1.begin(), cc1.end(), cc2.begin(), cc2.end()) ;

        stapl::sequential::get_cc_stats(localTree,cmap, ccs);
        cmap.reset();

      }
      //
      //
      //


      stats->StopClock(clockName.str());


    }

    VID GetClosestCC(VID _node, VID _nodeCCVID) const {

      RoadmapType* rdmp = m_problem->GetRoadmap();
      NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);

      stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
      vector< pair<size_t,VID> > ccs;
      stapl::sequential::get_cc_stats(*(rdmp->GetGraph()),cmap, ccs);

      typedef typename vector<pair<size_t, VID> >::iterator CCSIT;

      // Key = nodeVID, Value = CCVID
      // For easy retrieval of the CC once the closest node is found
      map<VID, VID> nodesAndCCs;
      typedef typename vector<pair<size_t, VID> >::iterator CCSIT;
      vector<VID> closestNodesOtherCCs;
      //find closest VID from other CCS
      for(CCSIT ccsit = ccs.begin(); ccsit!=ccs.end(); ccsit++){

        if(ccsit->second == _nodeCCVID)
          continue;

        vector<VID> cc;
        stapl::sequential::get_cc(*(rdmp->GetGraph()),cmap,ccsit->second,cc);
        cmap.reset();
        vector<VID> closest;
        nf->KClosest(rdmp, cc.begin(), cc.end(), _node, 1, back_inserter(closest));
        if (closest.size() != 0) { 
          closestNodesOtherCCs.push_back(closest[0]);
          nodesAndCCs[closest[0]] = ccsit->second; 
        }
      }

      //find closest VID from other CCS reps
      vector<VID> closestNode;
      nf->KClosest(rdmp, closestNodesOtherCCs.begin(), 
          closestNodesOtherCCs.end(), _node, 1, back_inserter(closestNode));

      VID closestCC = nodesAndCCs[ closestNode[0] ];

      return closestCC;
    }
    /*
       void ConnectCCs(vector<VID>& _cc1, vector<VID>& _cc2, typename GraphType::GRAPH _localTree) {


       }

       void Connect(LocalGraphType _localTree, vector<VID>::iterator _itr1First, vector<VID>::iterator  _itr1Last,
       vector<VID>::iterator _itr2First, vector<VID>::iterator _itr2Last) const {

       GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();

// Ta = itr1, Tb = itr2
vector<VID>* treeA = new vector<VID>();
vector<VID>* treeB = new vector<VID>();
for(InputIterator it = _itr1First; it != _itr1Last; ++it) {
treeA->push_back(*it);
}
for(InputIterator it = _itr2First; it != _itr2Last; ++it) {
treeB->push_back(*it);
}

// TODO All good till here
size_t iter = 0;
bool connected = false;
while( iter < m_iterations && !connected) {

CfgType dir = this->SelectDirection();
// Expand in direction of Ta
VID recent;
ExpandTree(dir, treeA, recent, m_delta);

if(recent != INVALID_VID) {
CfgType recentCfg = GetCfg(recent);

m_totalSuccess++;
treeA->push_back(recent);

VID newVID;

connected = ExpandTree(recentCfg, treeB, newVID, MAX_DBL);

} else {
int i = 0;
    // m_totalFailure++;
    }

// Switching trees
swap(treeA, treeB);
iter++;
}


if(this->m_debug) cout << "*** kClosest Time = " << _stats.GetSeconds("kClosest") << endl;
if(this->m_debug) cout << "*** m_totalSuccess = " << m_totalSuccess << endl;
if(this->m_debug) cout << "*** m_totalFailure = " << m_totalFailure << endl;

}


bool ExpandTree(LocalGraphType* _localTree, CfgType _dir, vector<VID>* _targetTree, VID& recentVID, double _delta) const {

// Setup MP Variables
Environment* env = m_problem->GetEnvironment();
StatClass* stats = m_problem->GetStatClass();
RoadmapType* rdmp = m_problem->GetRoadmap();
GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();
DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
NeighborhoodFinderPointer nf = m_problem->GetNeighborhoodFinder(m_nf);

recentVID = INVALID_VID;
CDInfo  cdInfo;
// Find closest Cfg in map
vector<VID> kClosest;
vector<CfgType> cfgs;

string kcloseClockName = "kclosest time ";
stats->StartClock(kcloseClockName);
// Choose the closest node from the three    
nf->KClosest(rdmp, _targetTree->begin(), _targetTree->end(), _dir, 1, back_inserter(kClosest));
kcloseStatClass->StopClock(kcloseClockName);

bool connected = false;

CfgType nearest = GetCfg(kClosest[0]);
CfgType newCfg;
int weight;

string expandClockName = "RRTConnect time ";
stats->StartClock(expandClockName);

string dmLabel = this->GetMPProblem()->GetNeighborhoodFinder(this->m_nfMethod)->GetDMMethod()->GetLabel();

bool expanded = RRTExpand<MPTraits>(this->GetMPProblem(), m_vcLabel, dmLabel, nearest, _dir, newCfg, 
    _delta, weight, cdInfo, env->GetPositionRes(), env->GetOrientationRes());

if(!expanded) {
  if(this->m_debug) cout << "RRT could not expand!" << endl; 
  return connected;
}
if (this->m_debug) cout<<"RRT expanded"<<endl;
expandStatClass->StopClock(expandClockName);
if(dm->Distance(env, newCfg, nearest) >= m_minDist) {
  // if newCfg = Dir, we reached goal
  if (newCfg == _dir && IsVertex(_dir))  {
    recentVID = GetVID(_dir);
    connected = true;
  } else {
    recentVID = rdmp->GetGraph()->AddVertex(newCfg);
  } 

  pair<WeightType, WeightType> weights = make_pair(WeightType("RRTConnect", weight), WeightType("RRTConnect", weight));
  AddEdge(kClosest[0], recentVID, weights);
  //rdmp->GetGraph()->GetCfg(recentVID).SetStat("Parent", kClosest[0]);
} 

return connected;
}
*/

};


template<class MPTraits>
class BuildRadialRRG {

  protected:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;

    MPProblemType* m_problem;
    size_t m_numNodes;
    string m_dmLabel;
    string m_vcLabel;
    string m_nfLabel;
    string m_ncLabel;
    double m_delta;
    double m_minDist;
    CfgType m_root;
    size_t m_numAttempts;
    bool m_strictBranching;
    size_t m_k;

  public:
    BuildRadialRRG(MPProblemType* _problem, size_t  _numNodes, string _dmLabel, string _vcLabel, string _nfLabel, 
        string _ncLabel, double _delta, double _minDist, CfgType _root, size_t _k, size_t _numAttempts, bool _strictBranching=false) {

      m_problem = _problem;
      m_numNodes = _numNodes;
      m_dmLabel = _dmLabel;
      m_delta = _delta;
      m_vcLabel = _vcLabel;
      m_nfLabel = _nfLabel;
      m_ncLabel = _ncLabel;
      m_minDist = _minDist;
      m_root = _root;
      m_numAttempts = _numAttempts;
      m_strictBranching = _strictBranching;
      // k-closest to connect, if k=0, then connect to ALL
      m_k = _k;
    }

    void define_type(stapl::typer& _t) {
      _t.member(m_numNodes);
      _t.member(m_problem);
      _t.member(m_dmLabel);
      _t.member(m_delta);
      _t.member(m_vcLabel);
      _t.member(m_nfLabel);
      _t.member(m_ncLabel);
      _t.member(m_minDist);
      _t.member(m_root);
      _t.member(m_numAttempts);
      _t.member(m_strictBranching);
      _t.member(m_k);
    }

    template<typename View> 
      void operator()(View _view) const
      {

        CfgType regionCand = _view.property().GetCandidate();
        vector<CfgType> neighbors = _view.property().GetNeighbors();

        ///Setup global variables
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric()->GetMethod(m_dmLabel);
        NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder()->GetMethod(m_nfLabel);
        Environment* env = m_problem->GetEnvironment();
        GraphType* globalTree = m_problem->GetRoadmap()->m_pRoadmap;
        vector<VID> branch;
        branch.clear();

        size_t ctr1=0;
        size_t maxAttempts = std::max(m_numNodes,m_numAttempts);
        while(branch.size()<m_numNodes && ctr1 < maxAttempts)
        { 

          CfgType dir, nearest, newCfg;
          VID nearestVID;

          double radius = dmm->Distance(env, m_root, regionCand);
          // No neighbors means only one region? 
          if(neighbors.size() == 0)
            dir.GetRandomCfg(env);
          else // TODO implement overlap
            dir = CfgType(); //SelectDirection(regionCand, neighbors, radius, m_overlap);



          /*At the first iteration, root is the only node in the tree so no need to call kclosest
            This prevent everyone banging on location 0 for root, however, if statement (branching) will be
            called m_numNodes times, let's hope branch prediction is smart enough to optimize it*/
          if(branch.size() == 0){
            nearestVID = 0;
            nearest = m_root;
          }else{

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
          if(RRTExpand<MPTraits>(m_problem,m_vcLabel, m_dmLabel, nearest, dir, newCfg, m_delta, weight, cdInfo,
                env->GetPositionRes(), env->GetOrientationRes())
              && (dmm->Distance(env, newCfg, nearest) >= m_minDist))
          {


            VID newVID = globalTree->add_vertex(newCfg);
            // TODO Fix VIZMO DEBUG
            VDAddNode(newCfg);
            branch.push_back(newVID);

            // After expanding, attempt connections to recent node
            vector<VID> currentVID;
            currentVID.push_back(newVID);

            ConnectorPointer pConnection;
            pConnection = m_problem->GetMPStrategy()->GetConnector()->GetMethod(m_ncLabel);    

            // Calling Connect Method and connecting nodes
            stapl::sequential::vector_property_map<typename GraphType::GRAPH,size_t > cmap;
            if (m_k == 0) {
              pConnection->Connect(m_problem->GetRoadmap(), 
                  *(m_problem->GetStatClass()), cmap,
                  currentVID.begin(), currentVID.end(), 
                  branch.begin(), branch.end());
            }
            else {
              vector<VID> kClosest;
              back_insert_iterator<vector<VID> > iterBegin(kClosest);
              nfp->KClosest((m_problem->GetRoadmap()),branch.begin(),branch.end(), dir, m_k, iterBegin); 

              pConnection->Connect(m_problem->GetRoadmap(), 
                  *(m_problem->GetStatClass()), cmap,
                  currentVID.begin(), currentVID.end(), 
                  kClosest.begin(), kClosest.end());

            }
          }

          ++ctr1;
        }
        PrintValue("Max Attempts : ", ctr1);
        PrintValue("Local Tree ", branch.size());
        _view.property().SetBranch(branch);

      }

};


template<class MPTraits>
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
    ConnectorPointer m_ncp;
    string m_dmLabel;

  public:
    ConnectRegionCCs(MPProblemType* _problem, ConnectorPointer _ncp, string _dmLabel) :m_problem(_problem){
      m_ncp = _ncp;
      m_dmLabel = _dmLabel;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_ncp);
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


        ColorMap cmap;
        vector< pair<size_t,VID> > localCCs;
        vector<VID> localCC;

        for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){

          RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();

          stapl::sequential::get_cc_stats(localTree ,cmap, localCCs);      

          int randCC = LRand() % localCCs.size();
          // Fill in the local CC
          stapl::sequential::get_cc(globalTree, cmap, localCCs[randCC].second, localCC);

          CfgType localccCentroid = GetCentroid(globalTree, localCC);
          ColorMap remoteCmap = tRegion.GetColorMap(); 
          vector< pair<size_t,VID> > remoteCCs = tRegion.GetCCs();
          VID closestRemoteCC = GetClosestCentroid(localccCentroid, remoteCCs, remoteCmap);

          // Fill in the remote CC
          vector<VID> remoteCC;
          stapl::sequential::get_cc(globalTree, remoteCmap, closestRemoteCC, remoteCC);

          /// NOW CONNECT
          ColorMap cmap2;
          m_ncp->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), cmap2,
              localCC.begin(),localCC.end(),
              remoteCC.begin(),remoteCC.end());

        }
      }

    // returns the VID of the centroid from the vec _ccs that is closest to _centroid
    VID GetClosestCentroid(CfgType _centroid, vector<pair<size_t, VID> > _ccs, ColorMap _cmap) const {
      DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dmLabel);
      Environment* env = m_problem->GetEnvironment();
      GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();


      vector<VID> cc;

      double currMinDist = MAX_DBL;
      VID currMinVID = INVALID_VID;

      for (int i=0; i<_ccs.size(); i++) {
        // TODO Cesar fix STAPL 
        stapl::sequential::get_cc(globalTree,_cmap,_ccs[i].second,cc);
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
};




#endif 
