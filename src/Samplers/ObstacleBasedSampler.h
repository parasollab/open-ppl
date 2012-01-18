#ifndef OBSTACLEBASEDSAMPLER_H_
#define OBSTACLEBASEDSAMPLER_H_

#include "SamplerMethod.h"
#include "Environment.h"
class Environment;
class StatClass;
class CDInfo;
class DistanceMetric;

template <typename CFG>
class ObstacleBasedSampler : public SamplerMethod<CFG> {
  private:
    string m_vcMethod, m_dmMethod;
    int m_nShellsFree, m_nShellsColl;
    double m_stepSize;
    bool m_useBBX;
    string m_pointSelection;		// This member variable is needed for the WOBPRM

  public:
    ObstacleBasedSampler() {
      this->SetName("ObstacleBasedSampler");
    }

    ObstacleBasedSampler(Environment* _env, int _free = 1, int _coll = 0, 
        double _step = 0, bool _useBBX=true, string _pointSelection="cspace")
      : m_nShellsFree(_free), m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX), m_pointSelection(_pointSelection){ 
        this->SetName("ObstacleBasedSampler");
        if(m_stepSize <= 0)
          m_stepSize = min(_env->GetPositionRes(), _env->GetOrientationRes());
      }

    ObstacleBasedSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("ObstacleBasedSampler");
      ParseXML(_node);
      Environment* env = _problem->GetEnvironment(); 
      if(m_stepSize <= 0.0)
        m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());
      if(this->m_debug) PrintOptions(cout);
    }

    ~ObstacleBasedSampler() {}

    void  ParseXML(XMLNodeReader& _node){
      XMLNodeReader::childiterator citr;
      m_useBBX = _node.boolXMLParameter("usebbx", true, false, "Use bounding box as obstacle");
      m_vcMethod = _node.stringXMLParameter("vc_method", true, "", "Validity Test Method");
      m_dmMethod =_node.stringXMLParameter("dm_method", true, "default", "Distance Metric Method");
      m_pointSelection = _node.stringXMLParameter("point_selection", false, "cspace", "point selection strategy");
      m_nShellsColl = _node.numberXMLParameter("n_shells_coll",true, 3,0,10, "Number of Col Shells");
      m_nShellsFree = _node.numberXMLParameter("n_shells_free",true, 3,0,10, "Number of Free Shells");
      m_stepSize = _node.numberXMLParameter("step_size",true, 0.0,0.0,10.0,
          "step size used in increment of cfg position towards or away from obstacles");
      
      // Checking if the read point_selection is valid or not
      if(!( m_pointSelection.compare("cspace")==0 || 
            m_pointSelection.compare("cM")==0 || m_pointSelection.compare("rV")==0 || m_pointSelection.compare("rT")==0 || 
            m_pointSelection.compare("rW")==0 || m_pointSelection.compare("eV")==0 || m_pointSelection.compare("rV_rT")==0 || 
            m_pointSelection.compare("rV_rW")==0 || m_pointSelection.compare("all")==0 )) {
        cerr << "Select a valid point selection type first. cspace, cM, rV ,rT, rW, eV, rV_rT, rV_rW, all are valid selection types. exiting.\n";
        exit(-1);
      }
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<CFG>::PrintOptions(_os);
      _os << "\tnShellsFree = " << m_nShellsFree << endl; 
      _os << "\tnShellsColl = " << m_nShellsColl << endl; 
      _os << "\tstepSize = " << m_stepSize << endl; 
      _os << "\tvcMethod = " << m_vcMethod << endl;
      _os << "\tdmMethod = " << m_dmMethod << endl;
      _os << "\tuseBBX = " << m_useBBX << endl; 
      _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
    }

    template <typename OutputIterator>
      OutputIterator GenerateShells(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats,
          CFG _cFree, CFG _cColl, CFG _incr, OutputIterator _result) {
        CDInfo cdInfo;
        string callee(this->GetName());
        callee += "::GenerateShells";
				ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
        // if(this->m_debug) cout << "m_nShellsColl = " << m_nShellsColl << endl;
        for(int i=0; i<m_nShellsFree; ++i) {
          if(_cFree.InBoundingBox(_env, _bb) && 
              vc->IsValid(vc->GetVCMethod(m_vcMethod), _cFree, _env, 
                _stats, cdInfo, true, &callee)) {
            _stats.IncNodesGenerated();
            *_result = _cFree;
            _result++;
          }
          _cFree.Increment(_incr);
        }

        CFG tmp;
        // if(this->m_debug) cout << "m_nShellsColl = " << m_nShellsColl << endl;
        _incr.subtract(tmp, _incr);
        for(int i=0; i<m_nShellsColl; ++i) {
          if(_cColl.InBoundingBox(_env, _bb) && 
              !vc->IsValid(vc->GetVCMethod(m_vcMethod), _cColl, _env, 
                _stats, cdInfo, true, &callee)){
            _stats.IncNodesGenerated();
            *_result = _cColl;
            _result++;
          }
          _cColl.Increment(_incr);
        }

        return _result;
      }

    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, 
        CFG& _cfgIn, vector<CFG>& _cfgOut, CFG& _cfgCol, int _maxAttempts) {
      string callee(this->GetName());
      callee += "::sampler()";
      CDInfo cdInfo;
      bool generated = false;

      int attempts = 0;
      do {
        _stats.IncNodesAttempted();
        attempts++;

        CFG c1 ;
        c1=ChooseASample( _cfgIn, _env, _bb);

        bool c1BBox = c1.InBoundingBox(_env, _bb);
				
        ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
        
	bool c1Free = vc->IsValid(vc->GetVCMethod(m_vcMethod), c1, _env, 
            _stats, cdInfo, true, &callee);

        CFG c2 = c1;
        bool c2BBox = c1BBox;
        bool c2Free = c1Free;

        CFG r;

	shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetDMMethod(m_dmMethod);

	r.GetRandomRay(m_stepSize, _env, dm);

        while(c2BBox && (c1Free == c2Free)) { 
          c1 = c2;
          c1BBox = c2BBox;
          c1Free = c2Free;

          c2.Increment(r);
          c2BBox = c2.InBoundingBox(_env, _bb);

          c2Free = vc->IsValid(vc->GetVCMethod(m_vcMethod), c2, _env, 
              _stats, cdInfo, true, &callee);

        }
        if(c2BBox) {
          generated = true;

          if(c1Free){
            CFG tmp;
            r.subtract(tmp, r);
            GenerateShells(_env, _bb, _stats,c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
            _cfgCol = c2;
          }
          else {
            GenerateShells(_env, _bb, _stats,c2, c1, r,back_insert_iterator<vector<CFG> >(_cfgOut));
            _cfgCol = c1;
          }
        }
        else if(c1BBox && m_useBBX && c1Free && !c2BBox){
          generated = true;
          CFG tmp;
          r.subtract(tmp, r);
          GenerateShells(_env, _bb, _stats,c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
          _cfgCol = c2;
        }
      } while (!generated && (attempts < _maxAttempts));

      return generated;
    }

    // Following is added after unification with WOPRM code
    CFG ChooseCenterOfMass(shared_ptr<MultiBody> _mBody) {
      Vector3D x = _mBody->GetCenterOfMass();
      CFG tmp;
      for(int i=0;i<3;i++)
        tmp.SetSingleParam(i, x[i]);
      for(int i=3;i<6;i++)
        tmp.SetSingleParam(i, 0.0);
      return tmp;
    }

    CFG ChooseRandomVertex(shared_ptr<MultiBody> _mBody,bool _isFreeBody) {
      GMSPolyhedron polyhedron;
      if(_isFreeBody) 
        polyhedron = _mBody->GetBody(0)->GetPolyhedron();
      else           
        polyhedron = _mBody->GetBody(0)->GetWorldPolyhedron();
      Vector3D x =polyhedron.vertexList[(int)(DRand()*polyhedron.vertexList.size())];

      CFG tmp;
      for(int i=0;i<3;i++)
        tmp.SetSingleParam(i, x[i]);
      for(int i=3;i<6;i++)
        tmp.SetSingleParam(i, 0.0);

      return tmp;
    }

    Vector3D ChoosePointOnTriangle(Vector3D _p, Vector3D _q, Vector3D _r) {
      Vector3D u, v;
      u = _q - _p;
      v = _r - _p;

      double s = DRand(); 
      double t = DRand();
      while(s + t > 1) {
        t = DRand();
      }
      return (_p + u*s + v*t);
    }

    CFG ChooseRandomWeightedTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron;
      if(_isFreeBody)
        polyhedron = _mBody->GetBody(0)->GetPolyhedron();
      else 
        polyhedron = _mBody->GetBody(0)->GetWorldPolyhedron();
      double area;
      area = _mBody->GetBody(0)->GetPolyhedron().area;

      double targetArea = area * DRand();

      int index, i;
      double sum;
      index = 0; 
      i = 1;
      sum = _mBody->GetBody(0)->GetPolyhedron().polygonList[0].area;
      while(targetArea > sum) {
        sum += _mBody->GetBody(0)->GetPolyhedron().polygonList[i].area;
        index++;
        i++;
      }

      // We choose the triangle of the _mBody with that index
      GMSPolygon *poly = &polyhedron.polygonList[index];

      // We choose a random point in that triangle
      Vector3D p, q, r;
      p = polyhedron.vertexList[poly->vertexList[0]];
      q = polyhedron.vertexList[poly->vertexList[1]];
      r = polyhedron.vertexList[poly->vertexList[2]];

      Vector3D x;
      x = ChoosePointOnTriangle(p, q, r);
      CFG tmp;
      for(int i=0;i<3;i++)
        tmp.SetSingleParam(i, x[i]);
      for(int i=3;i<6;i++)
        tmp.SetSingleParam(i, 0.0);
      return tmp;
    }

    CFG ChooseRandomTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron;
      if(_isFreeBody)
        polyhedron = _mBody->GetBody(0)->GetPolyhedron();
      else 
        polyhedron = _mBody->GetBody(0)->GetWorldPolyhedron();
      GMSPolygon *poly = &polyhedron.polygonList[(int)(DRand()*polyhedron.polygonList.size())];
      Vector3D p, q, r;
      p = polyhedron.vertexList[poly->vertexList[0]];
      q = polyhedron.vertexList[poly->vertexList[1]];
      r = polyhedron.vertexList[poly->vertexList[2]];
      Vector3D x;
      x = ChoosePointOnTriangle(p, q, r);
      CFG tmp;
      for(int i=0;i<3;i++)
        tmp.SetSingleParam(i, x[i]);
      for(int i=3;i<6;i++)
        tmp.SetSingleParam(i, 0.0);
      return tmp;
    }

    CFG ChooseExtremeVertex(shared_ptr<MultiBody> _mBody,bool _isFreeBody) {
      GMSPolyhedron polyhedron;
      // for robot, choose _mBody frame; for obstacle, choose world frame
      if(_isFreeBody)
        polyhedron = _mBody->GetBody(0)->GetPolyhedron();
      else 
        polyhedron = _mBody->GetBody(0)->GetWorldPolyhedron();

      int indexVert[6];
      for(int j = 0 ; j < 6 ; j++)
        indexVert[j] = 0;

      for(size_t i = 1 ; i < polyhedron.vertexList.size() ; i++) {
        //MAX X
        if(polyhedron.vertexList[i][0] < polyhedron.vertexList[indexVert[0]][0])
          indexVert[0] = i;

        //MIN X
        if(polyhedron.vertexList[i][0] > polyhedron.vertexList[indexVert[1]][0])
          indexVert[1] = i;

        //MAX Y
        if(polyhedron.vertexList[i][1] < polyhedron.vertexList[indexVert[2]][1])
          indexVert[2] = i;

        //MIN Y
        if(polyhedron.vertexList[i][1] > polyhedron.vertexList[indexVert[3]][1])
          indexVert[3] = i;

        //MAX Z
        if(polyhedron.vertexList[i][2] < polyhedron.vertexList[indexVert[4]][2])
          indexVert[4] = i;

        //<MIN Z
        if(polyhedron.vertexList[i][2] > polyhedron.vertexList[indexVert[5]][2])
          indexVert[5] = i;
      }

      // Choose an extreme random vertex at random
      int index = LRand() % 6;
      Vector3D x= polyhedron.vertexList[indexVert[index]];
      CFG tmp;
      for(int i=0;i<3;i++)
        tmp.SetSingleParam(i, x[i]);
      for(int i=3;i<6;i++)
        tmp.SetSingleParam(i, 0.0);
      return tmp;
    }

    shared_ptr<MultiBody> InitializeBody(Environment* _env) {
      int N =_env->GetMultiBodyCount();
      int roboindex = _env->GetRobotIndex(); 
      int obstacleIndex;
      do {
        obstacleIndex=LRand() % N;   
      } while(obstacleIndex==roboindex);
      return _env->GetMultiBody(obstacleIndex);// choose a multiBody randomly
    }

    virtual CFG ChooseASample(CFG _cfgIn, Environment* _env, shared_ptr<BoundingBox> _bb) {
      shared_ptr<MultiBody> mBody = InitializeBody(_env);
      bool isFreeBody = false;

      CFG temp;
      if(m_pointSelection.compare("cspace")==0 ){  // cspace is for Configuration space (This is for unifying OBPRM and WOBRM)
        temp =_cfgIn;
        if(temp == CFG())
          temp.GetRandomCfg(_env, _bb);//random configurations taken inside bounding box
      }
      else if(m_pointSelection.compare("cM")==0 )
        temp  = ChooseCenterOfMass(mBody);
      else if(m_pointSelection.compare("rV")==0 )
        temp  = ChooseRandomVertex(mBody,isFreeBody);
      else if(m_pointSelection.compare("rT")==0 )
        temp  = ChooseRandomTriangle(mBody,isFreeBody);
      else if(m_pointSelection.compare("rW")==0 )
        temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
      else if(m_pointSelection.compare("eV")==0 )
        temp  = ChooseExtremeVertex(mBody,isFreeBody);
      else if(m_pointSelection.compare("cM_rV")==0 ) { 
        int opt = LRand() % 2;
        if(opt == 0)
          temp  = ChooseCenterOfMass(mBody);
        else 
          temp  = ChooseRandomVertex(mBody,isFreeBody);
      }
      else if(m_pointSelection.compare("rV_rT")==0 ) {
        int opt = LRand() % 2;
        if(opt == 0)
          temp  = ChooseRandomTriangle(mBody,isFreeBody);
        else 
          temp  = ChooseRandomVertex(mBody,isFreeBody);
      }
      else if(m_pointSelection.compare("rV_rW")==0 ) {
        int opt = LRand() % 2;
        if(opt == 0)
          temp  = ChooseRandomVertex(mBody,isFreeBody);
        else
          temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
      }
      else if(m_pointSelection.compare("all")==0 ) {
        int opt = LRand() % 5;
        if(opt == 0)
          temp  = ChooseCenterOfMass(mBody);
        else if(opt == 1)
          temp  = ChooseRandomVertex(mBody,isFreeBody);
        else if(opt == 2)
          temp  = ChooseRandomTriangle(mBody,isFreeBody);
        else if(opt == 3)
          temp  = ChooseRandomWeightedTriangle(mBody,isFreeBody);
        else 
          temp  = ChooseExtremeVertex(mBody,isFreeBody);
      }
      else {
        cerr << "Select a valid point selection type first.exiting.\n";
        exit(-1);
      }

      return temp;
    }
};

#endif
