#ifndef CONNECTNEIGHBORINGSURFACES_H
#define CONNECTNEIGHBORINGSURFACES_H

#include "ConnectorMethod.h"

#include "Environment/SurfaceMultiBody.h"

#define CLOSEDIST 0.4
#define KATTEMPTS 3

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Connectors
/// @brief Connect between surfaces
/// @tparam MPTraits Motion planning universe
///
/// Connect between surfaces
/// for each s in surfaces
///    make sure boundary is built
///    for each other surface s' in surfaces
///       go through boundary lines, generate regular samples
///            and see if it's on that surface
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ConnectNeighboringSurfaces: public ConnectorMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::RoadmapType RoadmapType;
    typedef typename MPProblemType::VID VID;
    typedef typename vector<VID>::iterator VIDIT;

    //////////////////////
    // Constructors and Destructor
    ConnectNeighboringSurfaces(string _nf = "", string _lp = "",
        int _k = KATTEMPTS, string _surfacesStrToIgnore="");
    ConnectNeighboringSurfaces(MPProblemType* _problem, XMLNode& _node);
    virtual ~ConnectNeighboringSurfaces();

    //////////////////////
    // Used in new MPProblem framework.
    virtual void Print(ostream& _os) const;
    virtual void ParseXML(XMLNode& _node);

    //////////////////////
    // Core: Connection method
    template<typename InputIterator1, typename InputIterator2,
      typename OutputIterator>
        void Connect(RoadmapType* _rm,
            InputIterator1 _itr1First, InputIterator1 _itr1Last,
            InputIterator2 _itr2First, InputIterator2 _itr2Last,
            OutputIterator _collision);

  private:
    //////////////////////
    // Data
    int m_totalSuccess;
    int m_totalFailure;
    int m_k;
    bool m_doneOnce;
    string m_surfacesStrToIgnore;
    vector<string> m_surfacesToIgnore;

};

///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
ConnectNeighboringSurfaces<MPTraits>::
ConnectNeighboringSurfaces(string _nf, string _lp, int _k, string _surfacesStrToIgnore) :
  ConnectorMethod<MPTraits>(_nf, _lp), m_k(_k), m_surfacesStrToIgnore(_surfacesStrToIgnore) {
    this->SetName("ConnectNeighboringSurfaces");
    m_doneOnce = false;
    m_surfacesToIgnore = GetTags(m_surfacesStrToIgnore,",");
  }

///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
ConnectNeighboringSurfaces<MPTraits>::
ConnectNeighboringSurfaces(MPProblemType* _problem, XMLNode& _node) :
  ConnectorMethod<MPTraits>(_problem, _node),
  m_k(KATTEMPTS), m_doneOnce(false) {
    ParseXML(_node);
    m_surfacesToIgnore = GetTags(m_surfacesStrToIgnore,",");
  }

///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
ConnectNeighboringSurfaces<MPTraits>::
~ConnectNeighboringSurfaces(){
}

///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void ConnectNeighboringSurfaces<MPTraits>::
ParseXML(XMLNode& _node){
  this->SetName("ConnectNeighboringSurfaces");
  m_k = _node.Read("k", true, 0, 0, 10000, "k-value (max neighbors to find). k = 0 --> all-pairs");
  m_surfacesStrToIgnore = _node.Read("surfacesToIgnore", false, "", "surfaces to ignore (separated by , only [no spaces])");
}

///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
void ConnectNeighboringSurfaces<MPTraits>::Print(ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "    " << this->GetNameAndLabel() << "::  k = " << m_k << endl;
  _os << "\tsurfacesToIgnore = [";
  for(size_t i=0; i < m_surfacesToIgnore.size(); i++) {
    _os << m_surfacesToIgnore[i];
    if(i != m_surfacesStrToIgnore.size()-1) _os << ",";
  }
  _os << "]" << endl;
}

///////////////////////////////////////////////////////////////////////////////
// ConnectNeighboringSurfaces
// for each s \in surface
//     for each bl \in s->boundarLines()
//         for each specified division pt P along bl
//            does P represent valid connection between s and some surface
//            given a delta value (height difference)
//
///////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
template<typename InputIterator1, typename InputIterator2, typename OutputIterator>
void
ConnectNeighboringSurfaces<MPTraits>::
Connect(RoadmapType* _rm,
    InputIterator1 _itr1First, InputIterator1 _itr1Last,
    InputIterator2 _itr2First, InputIterator2 _itr2Last,
    OutputIterator _collision) {

  if(m_doneOnce) {
    if(this->m_debug){
      cout << " This connection need only be done once...BREAKING!" << endl;
    }
    return;
  }
  else {
    m_doneOnce=true;
  }

  m_totalSuccess = m_totalFailure = 0;

  Environment* env = this->GetEnvironment();
  int numSurfaces =  env->NumSurfaces();
  for(int i=0; i<numSurfaces; i++) {
    int id = i;
    shared_ptr<SurfaceMultiBody> mbSurf1 = env->GetSurface(i);
    string iSurfName="BASE";
    if( i>=0 )
      iSurfName=mbSurf1->GetLabel();;
    if(this->m_debug) cout << "ConnectNeighboringSurfaces::Connect(...) - " << i << "  Processing multibody: " << mbSurf1->GetLabel() << endl;
    if( find(m_surfacesToIgnore.begin(),m_surfacesToIgnore.end(), iSurfName) != m_surfacesToIgnore.end() ) continue;
    GMSPolyhedron& mbPoly1 =  mbSurf1->GetFixedBody(0)->GetWorldPolyhedron();
    vector<Vector3d>& ptsSurface1 = mbPoly1.GetVertexList();
    vector< pair<int,int> >& blines1 =  mbPoly1.GetBoundaryLines();

    if(this->m_debug) cout << " Number of boundary lines: " << blines1.size() << endl;
    for(size_t bl_i=0; bl_i<blines1.size(); bl_i++) {
      pair<int,int> line = blines1[bl_i];
      Vector3d v13d = ptsSurface1[line.first];
      Vector3d v23d = ptsSurface1[line.second];
      Point3d p13d(v13d[0],v13d[1],v13d[2]);
      Point3d p23d(v23d[0],v23d[1],v23d[2]);
      if(this->m_debug) cout << " edge: p1(3d): " << p13d << " p2(3d): " << p23d << " m_k: " << m_k << endl;
      Point2d p1(p13d[0],p13d[2]);
      Point2d p2(p23d[0],p23d[2]);
      Vector2d ldir = p2-p1;
      double length = ldir.norm();
      ldir = ldir.normalize(); // unit vector in direction of line
      Vector2d ldirP;
      ldirP[0] =    ldir[1];
      ldirP[1] = -1*ldir[0];

      for(int m=0; m<m_k; m++) {
	double prop;
	if( m_k==1 ) prop = 0.5;
	else {
	  double minL = 0.0;
	  double maxL = 1.0;
	  double remainL = maxL-minL;
	  double incrL = remainL/(m_k+1.0);
	  prop = (m+1)*incrL;
	}
	Point2d subPt = p1 + (prop*length) * ldir; //midpt of line
	//try points on either side of midpt to see if connected
	//to a connected surface
	Point2d subPtT1 = subPt + (0.5)*ldirP;
	Point2d subPtT2 = subPt + (-0.5)*ldirP;

	double h = (p13d[1]+p23d[1])/2.0; //surf->heightAtPt(subPt);
	Point2d queryPt, surfacePt;
	int qPtSurfID=-999;

	if( !mbPoly1.IsOnSurface( subPtT1, h ) ) {
	  queryPt = subPtT1;
	  surfacePt = subPtT2;
	}
	else {
	  queryPt = subPtT2;
	  surfacePt = subPtT1;
	}

	bool qPtOnSurf=false;
	//just do a height check
	bool skipBASE=false;
	string baseName="BASE";
	if(find(m_surfacesToIgnore.begin(),m_surfacesToIgnore.end(), baseName) != m_surfacesToIgnore.end()) skipBASE=true;
	if(!skipBASE && (fabs(h) < CLOSEDIST)) {
	  if(this->m_debug) cout << "queryPt: " << queryPt << " is on DEFAULT surface. " << endl;
	  qPtOnSurf = true;
	  qPtSurfID=-1;//default surface id
	}
	bool validOtherHeight = false;
	/////////////////////////////////////////////////////////////////////////////
	//loop over all surfaces
	for(int j=0; j<numSurfaces && !qPtOnSurf; j++) {
	  if(i==j) continue;//skip same surface..obvi
	  shared_ptr<SurfaceMultiBody> mbSurf2 = env->GetSurface(j);
	  string iSurfName2=mbSurf2->GetLabel();
	  if(i>=0)
	     iSurfName2=mbSurf2->GetLabel();;
          if( find(m_surfacesToIgnore.begin(),m_surfacesToIgnore.end(), iSurfName2) != m_surfacesToIgnore.end() ) continue;
	  if(this->m_debug) cout << "ConnectNeighboringSurfaces::Connect(...) - " << i << "  Processing multibody: " << j << " ::: " << mbSurf2->GetLabel() << endl;
	  GMSPolyhedron& mbPoly2 =  mbSurf2->GetFixedBody(0)->GetWorldPolyhedron();
	  //////////////////////////////////////////////////////////////////////////
	  //computation to check if things are connected.

	  int sid = j;
	  if( sid == -1 ) {
	    //just do a height check
	    if( fabs(h) < CLOSEDIST ) {
	      if(this->m_debug) cout << "queryPt: " << queryPt << " is on DEFAULT surface. " << endl;
	      qPtOnSurf=true;
	      qPtSurfID=sid;
	      break;
	    }
	  }
	  else {
	    if(this->m_debug) cout << " checking surface : " << mbSurf2->GetLabel() << endl;
	    if( mbPoly2.IsOnSurface(queryPt,h) ) {
	      double otherSurfHeight = mbPoly2.HeightAtPt(queryPt, validOtherHeight);
	      if(this->m_debug) cout << "On projected surface. otherSurfHeight: " << otherSurfHeight << " current height: " << h << endl;
	      if( fabs(otherSurfHeight-h)< CLOSEDIST ) {
		if(this->m_debug) cout << "queryPt: " << queryPt << " is on surface ID: " << mbSurf2->GetLabel() <<" from surface ID: "<< mbSurf1->GetLabel() << endl;
		qPtOnSurf=true;
		qPtSurfID=sid;
		break;
	      }
	    }
	    else {
	      if(this->m_debug) cout << " queryPt: " << queryPt << " not on surface: " << j <<endl;
	    }
	  }

	  //////////////////////////////////////////////////////////////////////////
	}//endfor j
	/////////////////////////////////////////////////////////////////////////////

	if( qPtOnSurf ) {
	  if(this->m_debug) cout << " found a valid connection between " << mbSurf1->GetLabel() << " id: " << id << " and surface id: " << qPtSurfID << endl;
	  //add in this node as a connection
	  //add pt on surface
          m_totalSuccess++;
	  CfgType cfg1(surfacePt[0], surfacePt[1], h, id);
	  VID vid1 = _rm->GetGraph()->AddVertex(cfg1);
	  CfgType cfg2(queryPt[0], queryPt[1], h, qPtSurfID);
	  VID vid2 = _rm->GetGraph()->AddVertex(cfg2);
	  double dist = (surfacePt-queryPt).norm();
	  if(this->m_debug) cout << " adding node: " << surfacePt << " and node: " << queryPt << endl;
	  if(this->m_debug) cout << " sampleBtwSurfaces added nid1: " << vid1 << " pt: "<< surfacePt << " surfID: " << id  << endl;
	  if(this->m_debug) cout << " sampleBtwSurfaces added nid2: " << vid2 << " pt: "<< queryPt << " surfID: " << qPtSurfID  << endl;

	  LPOutput<MPTraits> lpOutput;
	  lpOutput.m_edge.first.SetWeight(dist);
	  lpOutput.m_edge.second.SetWeight(dist);
	  _rm->GetGraph()->AddEdge(vid1, vid2, lpOutput.m_edge);

	}//endif qPtOnSurf
	else {
	  m_totalFailure++;
	  if(this->m_debug) cout << " NO valid connection for pt: " << surfacePt << " sid: " << id << endl;
	}
      }//forloop making subPts

      }//endfor bl_i < blines

    }//endfor i


    if(this->m_debug) {
      cout << "*** kClosest Time = " << this->GetStatClass()->GetSeconds("kClosest") << endl;
      cout << "*** m_totalSuccess = " << m_totalSuccess << endl;
      cout << "*** m_totalFailure = " << m_totalFailure << endl;
    }
  }

#endif
