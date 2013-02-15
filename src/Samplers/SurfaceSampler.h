#ifndef SURFACESAMPLER_H_
#define SURFACESAMPLER_H_

//#ifdef PMPCfgSurface

#include "SamplerMethod.h"


template <class MPTraits>
class SurfaceSampler : public SamplerMethod<MPTraits> {
  private:
    std::string m_vcLabel;
    int m_maxAttempts;
    double m_closeDist;
    double m_propForMA;
    double m_propForObs;
    int    m_obsSampleAttemptsPerIter;
    double m_minClearanceReq;
    double m_maxObsClearance;
    double m_minObsClearance;
    double m_overallMinClearanceReq;
    string m_surfacesStrToIgnore;
    vector<string> m_surfacesToIgnore;


  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceSampler(string _vcLabel="", double _maxAttempts=500, double _closeDist=0.1, double _propForMA=0.0, double _propForObs=0.0, int _obsSampleAttemptsPerIter=10, double _maxObsClearance=20.0, double _minObsClearance=2.0, double _overallMinClearanceReq=0.5, string _surfacesStrToIgnore="") 
      : m_vcLabel(_vcLabel), m_maxAttempts(_maxAttempts), m_closeDist(_closeDist), m_propForMA(_propForMA), m_propForObs(_propForObs), m_obsSampleAttemptsPerIter(_obsSampleAttemptsPerIter), m_maxObsClearance(_maxObsClearance), m_minObsClearance(_minObsClearance), m_overallMinClearanceReq(_overallMinClearanceReq), m_surfacesStrToIgnore(_surfacesStrToIgnore) {
      this->SetName("SurfaceSampler");
      m_surfacesToIgnore = GetTags(m_surfacesStrToIgnore,",");
    } 



    SurfaceSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem,_node) {
      this->SetName("SurfaceSampler");
      ParseXML(_node);
      m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
      m_maxAttempts = _node.numberXMLParameter("attempts",true, 500, 1,MAX_INT,"max attempts per surface"); 
      m_closeDist   = _node.numberXMLParameter("closeDist",false, 0.1, 0.0,MAX_DBL,"close distance to others"); 
      m_propForMA   = _node.numberXMLParameter("propForMA",false, 0.0, 0.0,1.0,"proportion [0..1] to push sample to medial axis"); 
      m_propForObs  = _node.numberXMLParameter("propForObs",false, 0.0, 0.0,1.0,"proportion [0..1] to generate sample near obs"); 
      m_obsSampleAttemptsPerIter = _node.numberXMLParameter("obsSampleAttempts",false, 20, 1,MAX_INT,"max attempts per surface to try obstacle samples"); 
      m_minObsClearance  = _node.numberXMLParameter("minObsClearance",false, 2.0, 0.0,MAX_DBL,"minimum obstacle clearance (when generating obs samples)"); 
      m_maxObsClearance  = _node.numberXMLParameter("maxObsClearance",false, 20.0, 0.0,MAX_DBL,"minimum obstacle clearance (when generating obs samples)"); 
      m_overallMinClearanceReq  = _node.numberXMLParameter("overallMinClearanceReq",false, 0.5, 0.0,MAX_DBL,"minimum clearance required from all samplers"); 
      m_surfacesStrToIgnore = _node.stringXMLParameter("surfacesToIgnore", false, "", "surfaces to ignore (separated by , only [no spaces])");


      m_surfacesToIgnore = GetTags(m_surfacesStrToIgnore,",");
    }

    ~SurfaceSampler() {}

    void ParseXML(XMLNodeReader& _node) {}

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<MPTraits>::PrintOptions(_out);
      _out << "\tm_vcLabel = " << m_vcLabel 
	<< "\tm_maxAttempts = " << m_maxAttempts 
	<< "\tm_closeDist = " << m_closeDist << endl;
      _out << "\tpropForMA = " << m_propForMA 
        << "\tpropForObs = " << m_propForObs 
	<< "\tobsSampleAttempts = " << m_obsSampleAttemptsPerIter << endl;
      _out << "\tminObsClearance = " << m_minObsClearance 
        << "\tmaxObsClearance = " << m_maxObsClearance
	<< "\toverallMinClearanceReq = " << m_overallMinClearanceReq << endl;
      _out << "\tsurfacesToIgnore = [";
      for(int i=0; i<(int)m_surfacesToIgnore.size();i++) {
	 _out << m_surfacesToIgnore[i];
	 if( i!=((int)m_surfacesStrToIgnore.size()-1)) _out << ",";
      }
      _out << "]" << endl;
    }

  protected:
    double DistToNearestOnSurf(CfgType& _cfg, vector<CfgType>& _cfgsV) {
      double closeD=1e6;
      Point2d pt = _cfg.GetPos();
      typedef typename vector<CfgType>::iterator CIT;
      for(CIT cit=_cfgsV.begin(); cit!=_cfgsV.end(); cit++){
	CfgType& tcfg = (*cit);
	Point2d tpt = tcfg.GetPos();
	double dist = (tpt-pt).norm();
	if( dist < closeD ) closeD=dist;
      }
      return closeD;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, 
        StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut, 
        vector<CfgType>& _cfgCol) { 
      string callee(this->GetName());
      callee += "::SampleImpl()";
      ValidityCheckerPointer vcp = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

      CDInfo cdInfo;

      int numSurfaces =  _env->GetNavigableSurfacesCount();
      int attempts = 0;
      SurfaceMedialAxisUtility<MPTraits> mau(this->GetMPProblem(), m_vcLabel, "Euclidean");
      for(int i=-1; i<numSurfaces; i++) {
	if( this->m_debug) cout << " Sampling for surface: " << i << endl;
	string iSurfName="BASE";
	if( i>=0 )
	   iSurfName=_env->GetNavigableSurface((size_t) i)->GetLabel();;
	if( find(m_surfacesToIgnore.begin(),m_surfacesToIgnore.end(), iSurfName) != m_surfacesToIgnore.end() ) continue; 
	attempts = 0;
	vector<CfgType> cfgsOnSurface;

	int iterForMA = m_propForMA * m_maxAttempts; 
	int iterForObs= m_propForObs* m_maxAttempts; 
	iterForObs+=iterForMA;
	if( this->m_debug) cout << " surface: " << i << " iterForMA: " << iterForMA << " iterForObs: " << iterForObs << endl;

	if(this->m_debug) VDClearAll();
	do {
	   bool validFound=false;
	  _stats.IncNodesAttempted(this->GetNameAndLabel());
	  attempts++;
	  CfgType tmp;
	  tmp.SetSurfaceID(i);
	  tmp.GetRandomCfg(_env,_bb);
	  double clear = MAX_DBL;
	  bool inBBX = tmp.InBoundary(_env, _bb);
	  if(this->m_debug) cout<<"tmp::"<<tmp<<" sid: "<< tmp.GetSurfaceID() << endl;
	  if(this->m_debug) cout<<"InBoundary::"<<inBBX<<endl;
	  if(inBBX) {
	    if(this->m_debug) cout << "vcp::name " << vcp->GetName() << endl;
	    bool isValid = vcp->IsValid(tmp, _env, _stats, cdInfo, &callee);
	    if(this->m_debug) cout << "IsValid::" << isValid << endl;
	    if(isValid) {
	      if( i==-1 ) {
		 if(iterForMA>attempts)  {
		    bool stillValid = false;
		    clear=mau.PushCfgToMedialAxis2DSurf(tmp, _bb, stillValid);
		    validFound=true;
		    if(this->m_debug) cout << " adding MA sample surf=-1" << endl;
		 }
		 else if(iterForObs>attempts) {
		    Point2d cdPt;
                    clear = mau.GetClearance2DSurf(_env, tmp.GetPos(), cdPt);
		    int j=0;
		    isValid=false; 
                    while( (!isValid || 
		           (clear>m_maxObsClearance) || 
		           (clear<m_minObsClearance)) && 
			   (j<m_obsSampleAttemptsPerIter) ) {
		       tmp.GetRandomCfg(_env,_bb);
		       isValid = vcp->IsValid(tmp, _env, _stats, cdInfo, &callee);
		       clear = mau.GetClearance2DSurf(_env, tmp.GetPos(), cdPt);
		       j++;
		    }

		    if( j<m_obsSampleAttemptsPerIter) {
		       validFound=true;
		       if(this->m_debug) cout << " adding Obs sample surf=-1 validFound: " << validFound <<" clear: " << clear << " j: " << j << endl;
		    }
		    else {
		       validFound=false;
		       if(this->m_debug) cout << " NOT adding Obs sample surf=-1 validFound: " << validFound <<" clear: " << clear << " j: " << j << endl;
		    }
		 }
		 else {
		    validFound=true; //this is the random case (which should be true)
		    Point2d cdPt;
                    clear = mau.GetClearance2DSurf(_env, tmp.GetPos(), cdPt);
		    if(this->m_debug) cout << " adding random sample surf=-1" << endl;
		 }
	      }
	      if( i>=0 ) { //all other surfaces
		 CfgType tmp2 = tmp;
		 //push to medial axis - setup
		 Point2d pos2d = tmp.GetPos();
		 double  h     = tmp.GetHeight();
		 Point3d pos3d(pos2d[0], h, pos2d[1]);
		 //push to medial axis - call
		 shared_ptr<MultiBody> surfi = _env->GetNavigableSurface((size_t) i);
		 shared_ptr<FixedBody> fb = surfi->GetFixedBody(0);
		 GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();

		 if( iterForMA>attempts ) {
		    clear=polyhedron.PushToMedialAxis(pos3d);
		    pos2d[0]=pos3d[0]; 
		    pos2d[1]=pos3d[2];
		    h       =pos3d[1];
		    tmp.SetPos(pos2d);
		    tmp.SetHeight(h);
		    validFound = vcp->IsValid(tmp, _env, _stats, cdInfo, &callee);
		    if( !validFound ) {
		       if(this->m_debug) cout << " after medial axis push, no longer valid...restoring old cfg." << endl;
		       tmp = tmp2;
		    }
		    else {
		       if(this->m_debug) cout << " adding MA sample surf=" <<i << endl;
		    }
		 }
		 else if(iterForObs>attempts) {
		    Point3d cdPt;
                    clear = polyhedron.GetClearance(pos3d, cdPt, -1);
		    int j=0; 
                    while( ((clear>m_maxObsClearance) || (clear<m_minObsClearance)) && (j<m_obsSampleAttemptsPerIter) ) {
		       tmp.GetRandomCfg(_env,_bb);
		       clear = polyhedron.GetClearance(pos3d, cdPt, -1);
		       j++;
		    }

		    if( j<m_obsSampleAttemptsPerIter) {
		       validFound=true;
		       pos2d[0]=pos3d[0]; 
		       pos2d[1]=pos3d[2];
		       h       =pos3d[1];
		       tmp.SetPos(pos2d);
		       tmp.SetHeight(h);
		       if(this->m_debug) cout << " adding Obs sample surf=" <<i << endl;
		    } 
		    else validFound=false;
		 }
		 else {
		    validFound=true; //this is the random case (which should be true)
		    Point3d cdPt;
                    clear = polyhedron.GetClearance(pos3d, cdPt, -1);
		 }
	      }

	      double d = DistToNearestOnSurf(tmp,cfgsOnSurface);
	      if( validFound && (d>m_closeDist) && (clear>m_overallMinClearanceReq) ) {
		if(this->m_debug) cout << "validFoundIndex::"<<_cfgOut.size()<<" cfg: "<<tmp << " closestNodeDist: " << d <<endl;
		_cfgOut.push_back(tmp);
		cfgsOnSurface.push_back(tmp);
	      }
	      else {
		if(this->m_debug) cout << "validFound but too close cfg: "<< tmp << " dist: " << d <<endl;
	      }
	    } else {
	      _cfgCol.push_back(tmp);
	    }
	  }

	  if(this->m_debug) cout << " Sampling for surface: " << i << " attempts: " << attempts << " of: " << m_maxAttempts << " validFound: " << validFound << endl;
	} while (attempts < m_maxAttempts);
      }//for i in surfaces
      if(this->m_debug) cout << "Maximum attempts reached." << endl;
      if(this->m_debug) cout << "SurfaceSampler:: done with sampling loop: size of cfgOut: " << _cfgOut.size() << endl;
      return true;
    } 

};

#endif
//#endif

