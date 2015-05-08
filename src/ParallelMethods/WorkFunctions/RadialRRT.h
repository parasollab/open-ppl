#ifndef RADIALRRT_H_
#define RADIALRRT_H_

#include "ParallelMethods/ParallelSBMPHeader.h"
#include "MPProblem/RoadmapGraph.h"
#include "MapReduceNF.h"
#include "RadialUtils.h"

using namespace psbmp;

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// Maintains a vector of size 2 with Min at the front and Max at the end
void PushMinMax(vector<double>& _vec, double _num) {
  // Empty Case
  if (_vec.size() == 0)
    _vec.push_back(_num);
  // Only one
  else if (_vec.size() == 1) {
    if (_vec.front() < _num)
      _vec.push_back(_num);
    else {
      _vec.push_back(_vec.front());
      _vec[0] = _num;
    }
  }
  // Compare and update if necessary
  else {
    if (_num < _vec[0])
      _vec[0] = _num;
    else if (_num > _vec[1])
      _vec[1] = _num;
  }
}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// from cartesion to spherical
template<class CfgType>
vector<double>
GetSphericalCoordinates(CfgType& _cfg) {
  vector<double> coordinates(3);
  double rho = 0;   // = sqrt(x^2 + y^2 + z^2)
  double theta = 0; // = arctan(y/x)
  double phi = 0;   // = arccos(z/rho)
  // Getting cartesian coordinates
  for (size_t j = 0; j < _cfg.PosDOF(); ++j) {
    coordinates[j] = _cfg[j];
    rho += pow(_cfg[j], 2.0);
  }
  // Coordinates = [X,Y,Z]
  rho = sqrt(rho);
  theta = atan2(coordinates[1],coordinates[0]);
  phi = MAX_INT;
  if(_cfg.PosDOF() == 3) {
    phi = acos(coordinates[2] / rho);

  }

  // Lets make all angles positive for more accurate comparison between quadrants, since atan2 returns [-2/pi,2/pi]
  while(theta < 0)
    theta += TWOPI;
  // from cartesian to polar
  coordinates[0] = rho;
  coordinates[1] = theta;
  coordinates[2] = phi;

  for (int i=_cfg.PosDOF(); i<_cfg.DOF(); i++)
    coordinates.push_back(2*DRand()-1.0);

  return coordinates;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// from spherical to cartesian
vector<double>
GetCartesianCoordinates(vector<double> sphericalCoordinates) {
  vector<double> coordinates(2);
  double rho = sphericalCoordinates[0];
  double theta = sphericalCoordinates[1];
  double phi = sphericalCoordinates[2];

  // from cartesian to polar
  coordinates[0] = rho * cos(theta) ;
  coordinates[1] = rho * sin(theta) ;

  // 3D case
  if(phi != MAX_INT) {
    coordinates[0] *= sin(phi) ;
    coordinates[1] *= sin(phi) ;
    coordinates.push_back(rho * cos(phi));
    for (int i=0; i<3; i++)
      coordinates.push_back(2*DRand()-1.0);
  }
  return coordinates;
}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class CfgType, class Environment>
CfgType
SelectDirection(Environment* _env, CfgType dir){
  if(dir == CfgType()){
    dir.GetRandomCfg(_env);
  }else{
    CfgType r;
    r.GetRandomCfg(_env);
    dir.subtract(dir,r);
  }
  return dir;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class CfgType>
CfgType
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {
  SelectDirection(_regionCand, _neighbors, _radius, 0);
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class CfgType>
CfgType
SelectDirection(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius, double _overlap) {
  CfgType dir = CfgType();

  // Store all the angles to get the max and min
  vector<double> thetas;
  vector<double> phis;

  if (dir.PosDOF() > 0) {

    vector<double> candCoordinates = GetSphericalCoordinates(_regionCand);
    vector<CfgType> midPoints = GetMiddlePoints(_regionCand, _neighbors, _radius);
    // TODO DEBUG only one neighbor, split region into halves
    if (midPoints.size() == 1) {
      CfgType point = -(midPoints[0]);
      midPoints.push_back(point);
    }

    for (size_t i = 0; i < midPoints.size(); ++i) {
      // [0] = Rho, [1] = Theta, [2] = Phi
      vector<double> neighborCoordinates;
      if(_neighbors.size() == 1)
        neighborCoordinates = GetSphericalCoordinates(_neighbors[0]);
      else
        neighborCoordinates = GetSphericalCoordinates(_neighbors[i]);

      vector<double> midPointCoordinates = GetSphericalCoordinates(midPoints[i]);
      vector<double> increments(3);

      // FIXME Increments not working, some angles are being shrinked
      // instead of enlarged
      increments[1] = (neighborCoordinates[1] - midPointCoordinates[1] ) * _overlap;
      midPointCoordinates[1] += increments[1];
      double increment = (neighborCoordinates[2] - midPointCoordinates[2]) * _overlap;
      midPointCoordinates[2] += increment;

      // We use this function to avoid pushing all angles and sorting them at the end
      PushMinMax(thetas, midPointCoordinates[1]);
      PushMinMax(phis, midPointCoordinates[2]);
    }

    // Is the range calculated correct? If cand theta is outside out [min,max] then fix the range to be [max-2PI, min]
    while (thetas[0] > candCoordinates[1] ) {
      double temp = thetas[0];
      thetas[0] = thetas[1] - TWOPI;
      thetas[1] = temp;
    }
    while (thetas[1] < candCoordinates[1]) {
      double temp = thetas[1];
      thetas[1] = thetas[0] + TWOPI;
      thetas[0] = temp;
    }
    // Randomizing
    // Rho = radius
    vector<double> randCoordinates(3);
    randCoordinates[0] = _radius  * sqrt(DRand());
    randCoordinates[1] = (thetas[1] - thetas[0]) * DRand() + thetas[0];
    randCoordinates[2] = MAX_INT;
    if(_regionCand.PosDOF() == 3)
      randCoordinates[2] = (phis[1] - phis[0]) * DRand() + phis[0];

    randCoordinates = GetCartesianCoordinates(randCoordinates);

    dir.SetData(randCoordinates);

    // Randomizing Rotational DOFs
    for (size_t i = dir.PosDOF(); i < dir.DOF(); i++)
      dir[i] = (2*DRand()-1.0);

    // TODO Fixed-Tree - I am not sure how to divide this space into regions.
  }else {
    for (size_t i=0; i < _regionCand.DOF(); i++) {
      vector<double> minMax;
      for (size_t j=0; j < _neighbors.size(); j++) {
        PushMinMax(minMax, _neighbors[j][i]);
      }
      dir[i] = (minMax.back() - minMax.front()) * DRand() + minMax.front();
    }
  }

  return dir;
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// Gets the point that is in between the candidate and the neighbor
template<class CfgType>
CfgType GetMiddlePoint(CfgType _regionCand, CfgType _neighbor, double _radius) {
  CfgType middlePoint;
  middlePoint = _regionCand + _neighbor;
  middlePoint = middlePoint/2;
  // Getting middle point across the circumference between candidate and neighbor
  vector<double> midPointCoordinates = GetSphericalCoordinates(middlePoint);
  midPointCoordinates[0] = _radius;
  midPointCoordinates = GetCartesianCoordinates(midPointCoordinates);

  middlePoint.SetData(midPointCoordinates);
  return middlePoint;

}


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
template<class CfgType>
vector<CfgType>
GetMiddlePoints(CfgType& _regionCand, vector<CfgType>& _neighbors, double _radius) {

  vector<CfgType> middlePoints;
  // Getting middle point across the circumference between candidate and neighbor
  for (size_t i = 0; i < _neighbors.size(); ++i)
    middlePoints.push_back(GetMiddlePoint(_regionCand, _neighbors[i], _radius));

  return middlePoints;

}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
/// @todo - Move to region class
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RadialRegion {
  public:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::GraphType GraphType;
    typedef typename MPTraits:: WeightType WeightType;
    typedef typename stapl::sequential::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, CfgType,WeightType> LocalGraphType;

    RadialRegion(CfgType _data = CfgType()) : m_data(_data) {}
    RadialRegion(const RadialRegion& _other)
      : m_problem(_other.m_problem), m_data(_other.m_data),
      m_neighbors(_other.m_neighbors), m_branch(_other.m_branch),
      m_ccs(_other.m_ccs) {}

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
    vector<CfgType> GetNeighbors() { return m_neighbors; }
    CfgType GetCandidate() { return m_data; }
    vector<VID> GetBranch() { return  m_branch;}
    WeightType GetWeight() { return  m_weight;}
    vector<pair<size_t, VID> > GetCCs() { return m_ccs;  }
    LocalGraphType GetLocalTree() { return m_localTree; }

    // Setters
    void SetCandidate(const CfgType& _data) { m_data= _data; }
    void SetNeighbors(const vector<CfgType>& _neighbors) { m_neighbors= _neighbors; }
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

  protected:
    MPProblemType* m_problem;
    CfgType m_data;
    vector<CfgType> m_neighbors;
    vector<VID> m_branch;
    WeightType  m_weight;
    vector<pair<size_t, VID> > m_ccs;
    LocalGraphType m_localTree;
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
      explicit proxy(Accessor const& _acc) : Accessor(_acc) { }
      operator target_t() const { return Accessor::read(); }
      proxy const& operator=(proxy const& _rhs) { Accessor::write(_rhs); return *this; }
      proxy const& operator=(target_t const& _rhs) { Accessor::write(_rhs); return *this;}

      CfgType GetCandidate() { return Accessor::invoke(&target_t::GetCandidate); }
      vector<CfgType> GetNeighbors() { return Accessor::invoke(&target_t::GetNeighbors); }
      vector<VID> GetBranch() { return Accessor::invoke(&target_t::GetBranch); }
      vector<pair<size_t, VID> > GetCCs() { return Accessor::invoke(&target_t::GetCCs); }
      WeightType GetWeight() { return Accessor::invoke(&target_t::GetWeight); }
      LocalGraphType GetLocalTree() { return Accessor::invoke(&target_t::GetLocalTree); }

      void SetCandidate(const CfgType _data) { Accessor::invoke(&target_t::SetCandidate, _data); }
      void SetNeighbors(const vector<CfgType> _neighbors) { Accessor::invoke(&target_t::SetNeighbors, _neighbors); }
      void SetBranch(const vector<VID> _branch) { Accessor::invoke(&target_t::SetBranch, _branch); }
      void SetWeight(const WeightType _weight) { Accessor::invoke(&target_t::SetWeight, _weight); }
      void SetMPProblem(const MPProblemType* _problem) {Accessor::invoke(&target_t::SetMPProblem, _problem); }
      void SetCCs(const vector<pair<size_t, VID> >& _ccs) { Accessor::invoke(&target_t::SetCCs, _ccs) ; }
      void SetLocalTree(const LocalGraphType _localTree) {Accessor::invoke(&target_t::SetLocalTree, _localTree); }

      void AddToBranch(const vector<VID> _branch) { Accessor::invoke(&target_t::AddToBranch, _branch); }
  }; //struct proxy
}

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    typedef void result_type;

    ConnectRegion(MPProblemType* _problem, ConnectorPointer _connector) :m_problem(_problem){
      m_connector = _connector;
    }

    void define_type(stapl::typer &_t){
      _t.member(m_problem);
      _t.member(m_connector);
    }

    template<typename vertexView, typename repeatView>
      result_type operator()(vertexView _view, repeatView& _gview)const {

        //typedef typename vertexView::adj_edges_type ADJV;
        //ADJV  edges = _view.edges();
        //SOURCE REGION
        vector<VID> sVids = _view.property().GetBranch();
        //TARGET REGION

        //for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
        for(typename vertexView::adj_edge_iterator ei = _view.begin(); ei != _view.end(); ++ei){
          RadialRegion<MPTraits> tRegion = (*(_gview.find_vertex((*ei).target()))).property();
          vector<VID> tVids = tRegion.GetBranch();

          /// NOW CONNECT
          m_connector->Connect(m_problem->GetRoadmap(),
              sVids.begin(),sVids.end(), tVids.begin(),tVids.end());

        }
      }

};



////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
class RadialRegionEdge {

  private:
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPTraits::WeightType WeightType;
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPProblemType::VID VID;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::NeighborhoodFinderPointer NeighborhoodFinderPointer;
    typedef pair<pair<VID, CfgType>, double> NFType;
    typedef vector<NFType> NFResultType;

    MPProblemType* m_problem;
    size_t  m_k;
    string m_dmLabel;

  public:
    typedef void result_type;

    typedef typename stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES, RadialRegion<MPTraits>, WeightType> RadialRegionGraph;
    RadialRegionEdge(MPProblemType* _problem, size_t _k, string _dmLabel):
      m_problem(_problem), m_k(_k), m_dmLabel(_dmLabel) {
      }

    void define_type(stapl::typer& _t) {
      _t.member(m_problem);
      _t.member(m_k);
      _t.member(m_dmLabel);
    }

    template <typename vertexView, typename repeatView>
      result_type operator() (vertexView _v1, repeatView _v2) {
        ///replace with call to NF
        DistanceMetricPointer dmm = m_problem->GetDistanceMetric(m_dmLabel);
        CfgType cfg = _v1.property().GetCandidate();
        VID vid = _v1.descriptor();
        vector<CfgType> neighbors;

        NFMapFuncRRRT<MPTraits> nfMap(m_problem, cfg, m_k, m_dmLabel);
        NFReduceFuncRRRT<MPTraits> nfReduce(m_k);
        NFResultType nfresult = map_reduce<skeletons::tags::with_coarsened_wf>(nfMap, nfReduce, _v2);

        //Add Edge between v and its k closest


        for(typename NFResultType::iterator itr = nfresult.begin(); itr != nfresult.end(); ++itr) {
          typename RadialRegionGraph::edge_descriptor ed1(vid, (*itr).first.first);
          //typename RadialRegionGraph::edge_descriptor ed2((*itr).first.first, vid);
          _v2.add_edge_async(ed1);
          //_v2.add_edge_async(ed2);
          neighbors.push_back((*itr).first.second);
          // TODO Vizmo Debug
          //VDAddEdge(cfg,neighbor);
        }
        _v1.property().SetNeighbors(neighbors);
      }


};


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
        CfgType point = m_root;
        point.GetRandomRay(m_radius, dmm);
        //point.GetRandomCfg(m_radius, m_radius);
        _vw.property().SetCandidate(point);
        //VDAddNode(point);
      }

};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
struct RadialRegionVertex2D{
  public:
    typedef typename MPTraits::CfgType CfgType;

    RadialRegionVertex2D(size_t _numRegions, CfgType _basePoint, double _radius):
      m_numRegions(_numRegions), m_basePoint(_basePoint), m_radius(_radius) {
      }

    void define_type(stapl::typer& _t){
      _t.member(m_basePoint);
      _t.member(m_radius);
      _t.member(m_numRegions);
    }

    /// Add point to region graph and have them evenly distributed
    typedef void result_type;
    template <typename View>
      result_type operator() (View _vw) const {
        double alpha = TWOPI/m_numRegions;
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

  private:
    // MPProblem* m_problem;
    double m_numRegions;
    CfgType m_basePoint;
    double m_radius;
};



////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

        double alpha = TWOPI/m_numRegions;
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



////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    typedef typename MPProblemType::ExtenderPointer ExtenderPointer;
    typedef typename MPProblemType::ConnectorPointer ConnectorPointer;
    typedef stapl::counter<stapl::default_timer> STAPLTimer;


    MPProblemType* m_problem;
    size_t m_numNodes;
    string m_dm;
    string m_vcm;
    string m_nfm;
    string m_eLabel;
    double m_delta;
    double m_minDist;
    CfgType m_root;
    size_t m_numAttempts;
    bool m_strictBranching;
    double m_overlap;

  public:
    BuildRadialRRT(MPProblemType* _problem, size_t _numNodes, string _dm, string _vcm, string _nfm, string _eLabel,
        double _delta, double _minDist, CfgType _root, size_t _numAttempts, const double _overlap, bool _strictBranching) {

      m_problem = _problem;
      m_numNodes = _numNodes;
      m_dm = _dm;
      m_delta = _delta;
      m_vcm = _vcm;
      m_nfm = _nfm;
      m_eLabel = _eLabel;
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
      _t.member(m_eLabel);
      _t.member(m_minDist);
      _t.member(m_root);
      _t.member(m_numAttempts);
      _t.member(m_strictBranching);
      _t.member(m_overlap);
    }

    typedef void result_type;
    template<typename View>
      result_type operator()(View _view) const {

        CfgType regionCand = _view.property().GetCandidate();
        vector<CfgType> neighbors = _view.property().GetNeighbors();

        ///Setup global variables
        DistanceMetricPointer dm = m_problem->GetDistanceMetric(m_dm);
        NeighborhoodFinderPointer nfp = m_problem->GetNeighborhoodFinder(m_nfm);
        ExtenderPointer e = m_problem->GetExtender(m_eLabel);
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

          double radius = dm->Distance(m_root, regionCand);
          // No neighbors means only one region?
          if(neighbors.size() == 0)
            dir.GetRandomCfg(env);
          else
            dir = SelectDirection(regionCand, neighbors, radius, m_overlap);

          /*At the first iteration, root is the only node in the tree so no need to call kclosest
            This prevent everyone banging on location 0 for root, however, if statement (branching) will be
            called m_numNodes times, let's hope branch prediction is smart enough to optimize it*/
          if(branch.empty()) {
            nearestVID = 0;
            nearest = m_root;
          }
          else {
            //Find closest from local tree
            vector<pair<VID, double>> kClosest;
            nfp->FindNeighbors(m_problem->GetRoadmap(),
                branch.begin(),branch.end(), dir, back_inserter(kClosest));
            nearestVID = kClosest[0].first;
          }
          // TODO DEBUG: do not add to root twice for each branch, remember to delete
          if (m_strictBranching && branch.size() > 1 && nearestVID == 0) continue;

          if (nearestVID == 0) {
            nearest = m_root;
          }
          else {
            // Hack to make sure nearest is always local
            // nearest = (*(globalTree->find_vertex(nearestVID))).property();
            nearest =   (*(globalTree->distribution().container_manager().begin()->find_vertex(nearestVID))).property();
          }

          //If expansion succeeds add to global tree and a copy in local tree

          LPOutput<MPTraits> lpOut;
          if(e->Extend(nearest, dir, newCfg, lpOut)
              && (dm->Distance(newCfg, nearest) >= m_minDist)) {

            VID newVID = globalTree->add_vertex(newCfg);
            //globalTree->AddEdge(nearestVID, newVID, lpOut.m_edge);
            globalTree->add_edge_async(nearestVID, newVID, lpOut.m_edge.first);

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





////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    typedef typename stapl::result_of::native_view<RoadmapViewType>::type NativeViewType;
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

          double radius = dm->Distance(m_root, regionCand);
          // No neighbors means only one region?
          if(neighbors.size() == 0)
            dir.GetRandomCfg(env);
          else
            dir = SelectDirection(regionCand, neighbors, radius, m_overlap);

          /*At the first iteration, root is the only node in the tree so no need to call kclosest
            This prevent everyone banging on location 0 for root, however, if statement (branching) will be
            called m_numNodes times, let's hope branch prediction is smart enough to optimize it*/
          if(branch.empty()) {
            nearestVID = 0;
            nearest = m_root;
          }
          else {
            //Find closest from local tree
            vector<pair<VID, double>> kClosest;
            nfp->FindNeighbors(m_problem->GetRoadmap(),
                branch.begin(),branch.end(), dir, back_inserter(kClosest));
            nearestVID = kClosest[0].first;
          }
          // TODO DEBUG: do not add to root twice for each branch, remember to delete
          if (m_strictBranching && branch.size() > 0 && nearestVID == 0)
            continue;

          if(nearestVID == 0) {
            nearest = m_root;
          }
          else {
            // Hack to make sure nearest is always local
            // nearest = (*(globalTree->find_vertex(nearestVID))).property();
            nearest =   (*(globalTree->distribution().container_manager().begin()->find_vertex(nearestVID))).property();
          }

          //If expansion succeeds add to global tree and a copy in local t
          //expandTree.start();
          //int samplesMade = radialUtils.ExpandTree(nearestVID, nearest, dir, pendingEdges, pendingWeights, expansionClk, process);
          /*int samplesMade =*/
          radialUtils.ExpandTree(branch, nearestVID, nearest, dir, pendingEdges, pendingWeights);
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
    double dist = dm->Distance(_centroid, otherCentroid);
    if (dist < currMinDist) {
      currMinDist = dist;
      currMinVID = _ccs[i].second;
    }

    cc.clear();
  }
  return currMinVID;
}

};*/

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename VID>
struct RegionCCs {
  typedef  vector<pair<size_t, VID> > result_type;
  template<typename P>
    result_type operator()(P& p) const{
      return p.GetCCs();
    }
};

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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


////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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
    typedef void result_type;

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
      result_type operator()(vertexView _view, repeatView& _gview)const {

        CfgType col;
        LPOutput<MPTraits> lpOutput;
        shared_ptr<RegionRRTConnect<MPTraits> > rrtConnect(dynamic_pointer_cast<RegionRRTConnect<MPTraits> >(m_connector));


        DistanceMetricPointer dm = m_problem->GetDistanceMetric("");
        //edges are assumed to be directed
        //typedef typename vertexView::adj_edges_type ADJV;
        //ADJV  edges = _view.edges();
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
        //for(typename vertexView::adj_edge_iterator ei = edges.begin(); ei != edges.end(); ++ei){
        for(typename vertexView::adj_edge_iterator ei = _view.begin(); ei != _view.end(); ++ei){
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

              vector<CfgType> col;
              if(rrtConnect->Connect(m_problem->GetRoadmap(),
                    localCC.begin(), localCC.end(),
                    remoteCC.begin(), remoteCC.end(),
                    back_inserter(col))) {
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

              vector<CfgType> col;
              if(rrtConnect->Connect(m_problem->GetRoadmap(),
                    localCC.begin(), localCC.end(),
                    remoteCC.begin(), remoteCC.end(),
                    back_inserter(col))) {
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



////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////
/// @ingroup ParallelMethods
/// @brief TODO
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename VID>
struct DeleteCC {
  typedef void result_type;

  DeleteCC(){}

  template<typename RepeatView>
    result_type operator()(const vector<VID>& v, RepeatView& _r) {
      for(auto&  i : v)
        _r.delete_vertex(i);
    }

  void define_type(stapl::typer& t) {
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
