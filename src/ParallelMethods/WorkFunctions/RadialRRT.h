//////////////////////////////////
//HEADER RadialRRT.h
/////////////////////////////////

#ifndef RADIALRRT_H_
#define RADIALRRT_H_

#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPProblem/MPTraits.h"
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
  typedef typename stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > ColorMap;
  typedef CfgType RegionType;
  typedef long unsigned int WeightType;

  protected:
  MPProblemType* m_problem;
  RegionType m_data;
  vector<RegionType> m_neighbors;
  vector<VID> m_branch;
  WeightType  m_weight;
  ColorMap m_colorMap;  // refreshed everytime GetCCs is called;
  vector<pair<size_t, VID> > m_ccs;

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
  ColorMap GetColorMap() { return m_colorMap; } 
  vector<pair<size_t, VID> > GetCCs() { return m_ccs;  }

  // Setters
  void SetCandidate(const RegionType& _data) { m_data= _data; }
  void SetNeighbors(const vector<RegionType>& _neighbors) { m_neighbors= _neighbors; }
  void SetBranch(const vector<VID>& _branch) {m_branch = _branch;}
  void SetWeight(const WeightType _weight) {m_weight = _weight;}
  void SetMPProblem(const MPProblemType* _problem) {m_problem = _problem;}
  void SetColorMap(const ColorMap& _colorMap) { m_colorMap = _colorMap; } 
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
   // _t.member(m_colorMap);
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
        void SetColorMap(const ColorMap& _colorMap) { Accessor::invoke(&target_t::SetColorMap, _colorMap); } 
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
          stapl::sequential::vector_property_map< typename GraphType::GRAPH ,size_t > colorMap;
          m_ncp->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), colorMap,
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
            // TODO Fix VIZMO DEBUG
//            if (this->m_debug) VDAddNode(newCfg);
            //TODO fix weight
            pair<WeightType, WeightType> weights = make_pair(WeightType("RRTExpand", weight), WeightType("RRTExpand", weight));
            //globalTree->AddEdge(nearestVID, newVID,weights);
            globalTree->add_edge_async(nearestVID, newVID,weights.first);
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
    typedef typename GraphType::GRAPH LocalGraphType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef typename stapl::graph_view<typename GraphType::GRAPH> RoadmapViewType;
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
    BuildRadialBlindRRT(MPProblemType* _problem, size_t _numNodes, string _dm, string _vc, string _nf, string _CCconnection, string _expansionType,
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
        StatClass* stats = m_problem->GetStatClass();
        GraphType* globalTree = m_problem->GetRoadmap()->GetGraph();

        // thes are used to keep work function small
        RadialUtils<MPTraits> radialUtils (m_problem, m_dm, m_vc, m_nf, m_CCconnection, m_expansionType, m_delta, m_minDist, m_numCCIters);
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
            kclosest.start();
            nfp->KClosest((m_problem->GetRoadmap()),branch.begin(),branch.end(), dir, 1, iterBegin); 
            kclosest.stop();
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
          expandTree.start();
          int samplesMade = radialUtils.ExpandTree(branch, nearestVID, nearest, dir, pendingEdges, pendingWeights, expansionClk, process);
          expandTree.stop();

          attempts++;

        }
        
        RoadmapViewType roadmapView  (*globalTree);
        typename RoadmapViewType::view_container_type& localTree = stapl::native_view(roadmapView)[get_location_id()].container();
        radialUtils.SetLocalTree(localTree);
        t1.stop();
        // I print the clocks all scattered because sometimes it gets stuck, so I can know where
       // PrintValue("Blind Build: ", t1.value() );
       // PrintValue("K Closest: ", kclosest.value() );
       // PrintValue("Expand Tree: ", expandTree.value() );
        // PrintValue("AddVertex Loop: ", expansionClk.value() );
        // PrintValue("AddEdge: ", process.value() );
        vector<STAPLTimer> timers;
        if (m_expansionType != "") { 
          t2.start(); 
          radialUtils.RemoveInvalidNodes(branch);
          t2.stop();
         // PrintValue("Remove Invalid: ",t2.value() );
          t3.start();
          
          
          for(int i = 0; i<5; i++) {
            STAPLTimer timer;
            timers.push_back(timer);
          }
          // TODO debug timer if need for using it!
          radialUtils.ConnectCCs(/*timers*/); 
          t3.stop();
          for(int i = 0; i<3; i++) {
         //   PrintValue("Connect CC Step: ", timers[i].value());
          }
         // PrintValue("Connect CCs: ", t3.value() );
        }
        /*t4.start();
        
        for (int i=0; i<pendingEdges.size(); i++) {
          WeightType weight ("RRTExpand", pendingWeights[i]);
          globalTree->add_edge(pendingEdges[i].first, pendingEdges[i].second, weight );
          globalTree->add_edge(pendingEdges[i].second, pendingEdges[i].first, weight );
        }
        
        t4.stop();*/
        // PrintValue("Pending Edges: ", t4.value() ); 


        t5.start();
        // setting CCs
        stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> colorMap;
        vector< pair<size_t,VID> > ccs;

        stapl::sequential::get_cc_stats(localTree,colorMap,ccs);
        _view.property().SetCCs(ccs);
        //Why are we setting branches?
        //_view.property().SetColorMap(colorMap);
        //_view.property().SetBranch(branch);
        
        t5.stop();
       // PrintValue("Setting CCs: ", t5.value() ); 
        t0.stop();
        
        PrintValue("Max Attempts : ", attempts);
        PrintValue("Local Tree ", branch.size());
        
         
       // PrintValue("Total: ", t0.value() ); 

      }

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
            //VDAddNode(newCfg);
            branch.push_back(newVID);

            // After expanding, attempt connections to recent node
            vector<VID> currentVID;
            currentVID.push_back(newVID);

            ConnectorPointer pConnection;
            pConnection = m_problem->GetMPStrategy()->GetConnector()->GetMethod(m_ncLabel);    

            // Calling Connect Method and connecting nodes
            stapl::sequential::vector_property_map<typename GraphType::GRAPH,size_t > colorMap;
            if (m_k == 0) {
              pConnection->Connect(m_problem->GetRoadmap(), 
                  *(m_problem->GetStatClass()), colorMap,
                  currentVID.begin(), currentVID.end(), 
                  branch.begin(), branch.end());
            }
            else {
              vector<VID> kClosest;
              back_insert_iterator<vector<VID> > iterBegin(kClosest);
              nfp->KClosest((m_problem->GetRoadmap()),branch.begin(),branch.end(), dir, m_k, iterBegin); 

              pConnection->Connect(m_problem->GetRoadmap(), 
                  *(m_problem->GetStatClass()), colorMap,
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
          m_ncp->Connect(m_problem->GetRoadmap(),*(m_problem->GetStatClass()), colorMap2,
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

    MPProblemType* m_problem;
    ConnectorPointer m_ncp;
    //LPOutput<MPTraits> lpOutput;
  public:
    ConnectGlobalCCs(MPProblemType* _problem, ConnectorPointer _ncp) :m_problem(_problem){
      m_ncp = _ncp;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_ncp);
    }


    template<typename vertexView, typename repeatView> 
      void operator()(vertexView _view, repeatView& _gview)const {
        
        CfgType col;
        Environment* env = m_problem->GetEnvironment();
        StatClass* stats = m_problem->GetStatClass();
        LPOutput<MPTraits> lpOutput;
        int weight;  
        //TODO : remove hardcoded DM
        DistanceMetricPointer dm = m_problem->GetDistanceMetric("euclidean");
        //edges are assumed to be directed
        typedef typename vertexView::adj_edges_type ADJV;
        ADJV  edges = _view.edges();
   
        //SOURCE REGION
        vector<pair<size_t,VID> > sCCs = _view.property().GetCCs();
        queue<VID> sPendingQ;
        for(typename vector<pair<size_t,VID> >::iterator it = sCCs.begin(); it != sCCs.end(); it++)
            sPendingQ.push((*it).second);
       
        //TARGET REGIONS

        for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
          RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();
          vector<pair<size_t,VID> > tCCs = tRegion.GetCCs();
          queue<VID> tPendingQ;
          vector<VID>  tConnected;
          stapl::sequential::vector_property_map<typename GraphType::GRAPH, size_t> cmap;
          
          for(typename vector<pair<size_t,VID> >::iterator tit = tCCs.begin(); tit != tCCs.end(); tit++)
            tPendingQ.push((*tit).second);
         
          while(!sPendingQ.empty()){
            VID sId = sPendingQ.front();
            sPendingQ.pop();  // really?
            for(typename vector<VID>::iterator vit = tConnected.begin(); vit != tConnected.end(); vit++){
                if(m_problem->GetLocalPlanner("sl")->
                   IsConnected(env,*stats, dm,
                   m_problem->GetRoadmap()->GetGraph()->GetCfg(sId),
                   m_problem->GetRoadmap()->GetGraph()->GetCfg(*vit),
                   col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes(), 
                   (false) )){
                     // if connection was made, add edge and break
                     //m_problem->GetRoadmap()->GetGraph()->AddEdge(sId, *vit, lpOutput.edge);
                     pair<WeightType, WeightType> weights = make_pair(WeightType("GlobalConnect", weight), WeightType("GlobalConnect", weight));
                     //globalTree->AddEdge(nearestVID, newVID,weights);
                     m_problem->GetRoadmap()->GetGraph()->add_edge_async(sId, *vit,weights.first);
                     break;
                }
             }
             while(!tPendingQ.empty()){
               VID tId = tPendingQ.front();
               tPendingQ.pop(); /// how do you return a value and remove it in one line?
               ///TODO: Remove hard-coded sl lp
               if(m_problem->GetLocalPlanner("sl")->
                   IsConnected(env,*stats, dm,
                   m_problem->GetRoadmap()->GetGraph()->GetCfg(sId),
                   m_problem->GetRoadmap()->GetGraph()->GetCfg(tId),
                   col, &lpOutput, env->GetPositionRes(), env->GetOrientationRes(), 
                   (false) )){
                     // if connection was made, add edge and increment remote connected set 
                     //m_problem->GetRoadmap()->GetGraph()->AddEdge(sId, tId, lpOutput.edge);
                     pair<WeightType, WeightType> weights = make_pair(WeightType("GlobalConnect", weight), WeightType("GlobalConnect", weight));
                     m_problem->GetRoadmap()->GetGraph()->add_edge_async(sId, tId,weights.first);
                     tConnected.push_back(tId);
               }

             }

          }

        }
      }

};


#endif 
