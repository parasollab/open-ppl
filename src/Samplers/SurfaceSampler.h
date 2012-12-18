#ifndef SURFACESAMPLER_H_
#define SURFACESAMPLER_H_

#include "SamplerMethod.h"


template <class MPTraits>
class SurfaceSampler : public SamplerMethod<MPTraits> {
  private:
    std::string m_vcLabel;
    int m_maxAttempts;
    double m_closeDist;

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    SurfaceSampler(string _vcLabel="", double _maxAttempts=500, double _closeDist=0.1) 
      : m_vcLabel(_vcLabel), m_maxAttempts(_maxAttempts), m_closeDist(_closeDist) {
      this->SetName("SurfaceSampler");
    } 


    SurfaceSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem,_node) {
      this->SetName("SurfaceSampler");
      ParseXML(_node);
      m_vcLabel = _node.stringXMLParameter("vcMethod", true, "", "Validity Test Method");
      m_maxAttempts = _node.numberXMLParameter("attempts",true, 500, 1,MAX_INT,"max attempts per surface"); 
      m_closeDist = _node.numberXMLParameter("closeDist",false, 0.1, 0.0,MAX_DBL,"close distance to others"); 
    }

    ~SurfaceSampler() {}

    void ParseXML(XMLNodeReader& _node) {}

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<MPTraits>::PrintOptions(_out);
      _out << "\tm_vcLabel = " << m_vcLabel 
	<< "\tm_maxAttempts = " << m_maxAttempts 
	<< "\tm_closeDist = " << m_closeDist << endl;
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
#ifdef PMPCfgSurface
      string callee(this->GetName());
      callee += "::SampleImpl()";
      ValidityCheckerPointer vcp = this->GetMPProblem()->GetValidityChecker(m_vcLabel);

      CDInfo cdInfo;

      double propForMA=1.0; 
      int numSurfaces =  _env->GetNavigableSurfacesCount();
      bool validFound=false;
      int attempts = 0;
      for(int i=-1; i<numSurfaces; i++) {
	if( this->m_debug) cout << " Sampling for surface: " << i << endl;
	attempts = 0;
	vector<CfgType> cfgsOnSurface;

	if(this->m_debug) VDClearAll();
	do {
	  _stats.IncNodesAttempted(this->GetNameAndLabel());
	  attempts++;
	  CfgType tmp;
	  tmp.SetSurfaceID(i);
	  tmp.GetRandomCfg(_env,_bb);
	  bool inBBX = tmp.InBoundary(_env, _bb);
	  if(this->m_debug) cout<<"tmp::"<<tmp<<" sid: "<< tmp.GetSurfaceID() << endl;
	  if(this->m_debug) cout<<"InBoundary::"<<inBBX<<endl;
	  if(inBBX) {
	    bool isValid = vcp->IsValid(tmp, _env, _stats, cdInfo, &callee);
	    if(this->m_debug) cout << "IsValid::" << isValid << endl;
	    if(isValid) {
	      double maProb = DRand();
	      if( i>=0 && maProb < propForMA ) {
		CfgType tmp2 = tmp;
		//push to medial axis - setup
		Point2d pos2d = tmp.GetPos();
		double  h     = tmp.GetHeight();
		Point3d pos3d(pos2d[0], h, pos2d[1]);
		//push to medial axis - call
		shared_ptr<MultiBody> surfi = _env->GetNavigableSurface((size_t) i);
		shared_ptr<FixedBody> fb = surfi->GetFixedBody(0);
		GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();
		polyhedron.PushToMedialAxis(pos3d);
		pos2d[0]=pos3d[0]; pos2d[1]=pos3d[2];
		h       =pos3d[1];
		tmp.SetPos(pos2d);
		tmp.SetHeight(h);
		isValid = vcp->IsValid(tmp, _env, _stats, cdInfo, &callee);
		if( !isValid ) {
		  if(this->m_debug) cout << " after medial axis push, no longer valid...restoring old cfg." << endl;
		  tmp = tmp2;
		}
	      }
	      double d = DistToNearestOnSurf(tmp,cfgsOnSurface);
	      if( d>m_closeDist) {
		validFound = true;
		if(this->m_debug) cout << "validFound::"<<validFound<<" cfg: "<<tmp<<endl;
		_cfgOut.push_back(tmp);
		cfgsOnSurface.push_back(tmp);
	      }
	    } else {
	      _cfgCol.push_back(tmp);
	    }
	  }

	  if(this->m_debug) cout << " Sampling for surface: " << i << " attempts: " << attempts << " of: " << m_maxAttempts << " validFound: " << validFound << endl;
	} while (attempts < m_maxAttempts);
      }//for i in surfaces
      if(this->m_debug) cout << "Maximum attempts reached." << endl;
      if(this->m_debug) cout << "SurfaceSampler:: done with sampling loop: " << validFound << " size of cfgOut: " << _cfgOut.size() << endl;
#endif //defined PMPCfgSurface
      return true;
    } 

};

#endif

