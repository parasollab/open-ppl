// OBPRM samples by pushing random configurations along a random ray
// until they change validity, keeping the best free configuration

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
    
    string m_vcMethod, m_dmMethod; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM

    // Returns a CFG with the coordinates specified in the vector and no rotation
    CFG GetCfgWithParams(Vector3D& _v) {
      CFG tmp;
      for(int i = 0; i < 3; i++)
        tmp.SetSingleParam(i, _v[i]);
      for(int i = 3; i < 6; i++)
        tmp.SetSingleParam(i, 0.0);
      return tmp;
    }
 
    // Returns an appropriate polygon: for a robot, the _mBody frame; for an obstacle, the world frame
    GMSPolyhedron GetPolyhedron(shared_ptr<MultiBody>& _mBody, bool _isFreeBody) {
      if(_isFreeBody)
        return _mBody->GetBody(0)->GetPolyhedron();
      else
        return _mBody->GetBody(0)->GetWorldPolyhedron();
    }

  public:

    ObstacleBasedSampler(Environment* _env = NULL, string _vcMethod = "", string _dmMethod = "",
        int _free = 1, int _coll = 0, double _step = 0, bool _useBBX = true, string _pointSelection = "cspace")
      : m_vcMethod(_vcMethod), m_dmMethod(_dmMethod), m_nShellsFree(_free), m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX), m_pointSelection(_pointSelection) { 
      this->SetName("ObstacleBasedSampler");
      // If the step size is unreasonable, set it to the minimum
      if(m_stepSize <= 0.0)
        if(_env != NULL)
          m_stepSize = min(_env->GetPositionRes(), _env->GetOrientationRes());
    }

    ObstacleBasedSampler(XMLNodeReader& _node, MPProblem* _problem) : SamplerMethod<CFG>(_node, _problem) {
      this->SetName("ObstacleBasedSampler");
      ParseXML(_node);
      Environment* env = _problem->GetEnvironment(); 
      // If the step size is unreasonable, set it to the minimum
      if(m_stepSize <= 0.0)
        m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());
      if(this->m_debug)
        PrintOptions(cout);
    }

    ~ObstacleBasedSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_useBBX = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
      m_vcMethod = _node.stringXMLParameter("vcMethod", true, "", "Validity test method");
      m_dmMethod =_node.stringXMLParameter("dmMethod", true, "default", "Distance metric method");
      m_pointSelection = _node.stringXMLParameter("pointSelection", false, "cspace", "Point selection strategy");
      m_nShellsColl = _node.numberXMLParameter("nShellsColl", true, 3, 0, 10, "Number of collision shells");
      m_nShellsFree = _node.numberXMLParameter("nShellsFree", true, 3, 0, 10, "Number of free shells");
      m_stepSize = _node.numberXMLParameter("stepSize", true, 0.0, 0.0, 10.0,
          "Step size used in increment of cfg position towards or away from obstacles");
      _node.warnUnrequestedAttributes();

      // Check if the read point_selection is valid
      if(m_pointSelection != "cspace" && m_pointSelection != "cM" && m_pointSelection != "rV" &&
             m_pointSelection != "rT" && m_pointSelection != "rW" && m_pointSelection != "eV" &&
             m_pointSelection != "rV_rT" && m_pointSelection != "rV_rW" && m_pointSelection != "all") {
        cerr << "Select a valid point selection type first.\
          cspace, cM, rV ,rT, rW, eV, rV_rT, rV_rW, and all are valid selection types. Exiting." << endl;
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

    // Generates and adds shells to their containers
    template <typename OutputIterator>
    OutputIterator GenerateShells(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats,
          CFG _cFree, CFG _cColl, CFG _incr, OutputIterator _result) {
      
      string callee = this->GetNameAndLabel() + "::GenerateShells()";
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      CDInfo cdInfo;
      if(this->m_debug)
        cout << "nShellsColl = " << m_nShellsColl << endl;
    
      // Add free shells
      for(int i = 0; i < m_nShellsFree; i++) {
        // If the shell is valid
        if(_cFree.InBoundingBox(_env, _bb) && 
            vc->IsValid(vc->GetVCMethod(m_vcMethod), _cFree, _env, _stats, cdInfo, true, &callee)) {
          if(this->m_recordKeep)
            _stats.IncNodesGenerated(this->GetNameAndLabel());
          // Add shell
          *_result = _cFree;
          _result++;
        }
        // Get next shell
        _cFree.Increment(_incr);
      }

      // Reverse direction of _incr
      CFG tmp;
      _incr.subtract(tmp, _incr);
      
      // Add collision shells
      for(int i = 0; i < m_nShellsColl; i++) {
        // If the shell is valid
        if(_cColl.InBoundingBox(_env, _bb) && 
            !vc->IsValid(vc->GetVCMethod(m_vcMethod), _cColl, _env, _stats, cdInfo, true, &callee)) {
          if(this->m_recordKeep)
            _stats.IncNodesGenerated(this->GetNameAndLabel());
          // Add shell
          *_result = _cColl;
          _result++;
        }
        // Get next shell
        _cColl.Increment(_incr);
      }
      return _result;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<BoundingBox> _bb, StatClass& _stats, 
        CFG& _cfgIn, vector<CFG>& _cfgOut, vector<CFG>& _cfgCol) {

      string callee = this->GetNameAndLabel() + "::Sampler()";
      ValidityChecker<CFG>* vc = this->GetMPProblem()->GetValidityChecker();
      shared_ptr<DistanceMetricMethod> dm = this->GetMPProblem()->GetDistanceMetric()->GetMethod(m_dmMethod);
      CDInfo cdInfo;

      if(this->m_recordKeep)
        _stats.IncNodesAttempted(this->GetNameAndLabel());

      // Old state
      CFG c1 = ChooseASample(_cfgIn, _env, _bb);
      bool c1BBox = c1.InBoundingBox(_env, _bb);  
      bool c1Free = vc->IsValid(vc->GetVCMethod(m_vcMethod), c1, _env, _stats, cdInfo, true, &callee);

      // New state
      CFG c2 = c1;
      bool c2BBox = c1BBox;
      bool c2Free = c1Free;

      // Create a random ray
      CFG r;
      r.GetRandomRay(m_stepSize, _env, dm);
      if(r == CFG()) {
        if(this->m_debug)
          cerr << "Random ray in OBPRM Sampler is 0-valued! Continuing with loop." << endl;
        return false;
      }

      // Loop until the new state is outside the bounds or the validity changes
      while(c2BBox && (c1Free == c2Free)) { 
        // Copy new data to old state
        c1 = c2;
        c1BBox = c2BBox;
        c1Free = c2Free;
        // Update new state
        c2.Increment(r);
        c2BBox = c2.InBoundingBox(_env, _bb);
        c2Free = vc->IsValid(vc->GetVCMethod(m_vcMethod), c2, _env, _stats, cdInfo, true, &callee);
      }

      // If new state is in BBox (there must be a validity change)
      if(c2BBox) {
        if(c1Free) { // Old state (c1) is free
          // Reverse direction of r
          CFG tmp;
          r.subtract(tmp, r);
          // Process configurations
          GenerateShells(_env, _bb, _stats, c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
          _cfgCol.push_back(c2);
        }
        else { // New state (c2) is free
          // Process configurations
          GenerateShells(_env, _bb, _stats, c2, c1, r,back_insert_iterator<vector<CFG> >(_cfgOut));
          _cfgCol.push_back(c1);
        }
        return true;
      }
      else if(m_useBBX && c1BBox && c1Free) {
        // Reverse direction of r
        CFG tmp;
        r.subtract(tmp, r);
        // Process configurations
        GenerateShells(_env, _bb, _stats, c1, c2, r, back_insert_iterator<vector<CFG> >(_cfgOut));
        _cfgCol.push_back(c2);
        return true;
      }
      return false;
    }

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CFG at the center of mass of the MultiBody
    CFG ChooseCenterOfMass(shared_ptr<MultiBody> _mBody) {
      Vector3D x = _mBody->GetCenterOfMass();
      return GetCfgWithParams(x);
    }

    // Returns a CFG at a random vertex of the MultiBody
    CFG ChooseRandomVertex(shared_ptr<MultiBody> _mBody, bool _isFreeBody) { 
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      Vector3D x = polyhedron.m_vertexList[(int)(DRand()*polyhedron.m_vertexList.size())];
      return GetCfgWithParams(x);
    }

    // Returns a point inside the triangle determined by the vectors
    Vector3D ChoosePointOnTriangle(Vector3D _p, Vector3D _q, Vector3D _r) {
      Vector3D u = _q - _p; // From _p to _q
      Vector3D v = _r - _p; // From _p to _r
      double s = DRand(); 
      double t = DRand();
      // Keep point inside the triangle
      while(s + t > 1) {
        s = DRand();
        t = DRand();
      }
      return (_p + u*s + v*t);
    }

    // Chooses a random point on a random triangle (weighted by area) in the MultiBody
    CFG ChooseRandomWeightedTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      // A random fraction of the area
      double targetArea = _mBody->GetBody(0)->GetPolyhedron().m_area * DRand();
      double sum = 0.0;
      int index;
      
      // Choose index as the triangle that first makes sum > targetArea
      for(index = -1; sum <= targetArea; index++)
        sum += _mBody->GetBody(0)->GetPolyhedron().m_polygonList[index + 1].m_area;
      // Choose the triangle of the MultiBody with that index
      GMSPolygon *poly = &polyhedron.m_polygonList[index];
      // Choose a random point in that triangle
      Vector3D p = polyhedron.m_vertexList[poly->m_vertexList[0]];
      Vector3D q = polyhedron.m_vertexList[poly->m_vertexList[1]];
      Vector3D r = polyhedron.m_vertexList[poly->m_vertexList[2]];
      Vector3D x = ChoosePointOnTriangle(p, q, r);
      return GetCfgWithParams(x);
    }

    // Chooses a random point in a random triangle in the MultiBody
    CFG ChooseRandomTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      // Choose a random triangle
      GMSPolygon *poly = &polyhedron.m_polygonList[(int)(DRand()*polyhedron.m_polygonList.size())];
      Vector3D p = polyhedron.m_vertexList[poly->m_vertexList[0]];
      Vector3D q = polyhedron.m_vertexList[poly->m_vertexList[1]];
      Vector3D r = polyhedron.m_vertexList[poly->m_vertexList[2]];
      Vector3D x = ChoosePointOnTriangle(p, q, r);
      return GetCfgWithParams(x);
    }

    // Chooses a random extreme vertex of the MultiBody
    CFG ChooseExtremeVertex(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      int xyz = LRand() % 3; // 0: x, 1: y, 2: z
      int minMax = LRand() % 2; // 0: min, 1: max
      int x = 0; // Index of extreme value
      
      // Find extreme value
      for(size_t i = 1; i < polyhedron.m_vertexList.size(); i++)
        if(((polyhedron.m_vertexList[i][xyz] < polyhedron.m_vertexList[x][xyz]) + minMax) % 2) // minMax is an optional negation
          x = i;
      return GetCfgWithParams(polyhedron.m_vertexList[x]);
    }

    // Chooses an obstacle MultiBody randomly
    shared_ptr<MultiBody> InitializeBody(Environment* _env) {
      int N = _env->GetMultiBodyCount();
      int roboIndex = _env->GetRobotIndex(); 
      int obstacleIndex;
      
      if(this->m_debug && N == 1)
        cout << "Infinite loop in InitializeBody() of ObstacleBasedSampler.h because N == 1" << endl;
      do {
        obstacleIndex = LRand() % N;   
      } while(obstacleIndex == roboIndex);
      return _env->GetMultiBody(obstacleIndex);
    }

    // Checks m_pointSelection and returns an appropriate CFG
    virtual CFG ChooseASample(CFG _cfgIn, Environment* _env, shared_ptr<BoundingBox> _bb) {
      shared_ptr<MultiBody> mBody = InitializeBody(_env);
      // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
      if(m_pointSelection == "cspace") {  
        if(_cfgIn == CFG())
          // Get random configuration inside bounding box
          _cfgIn.GetRandomCfg(_env, _bb);
        return _cfgIn;
      }
      else if(m_pointSelection == "cM")
        return ChooseCenterOfMass(mBody);
      else if(m_pointSelection == "rV")
        return ChooseRandomVertex(mBody, false);
      else if(m_pointSelection == "rT")
        return ChooseRandomTriangle(mBody, false);
      else if(m_pointSelection == "rW")
        return ChooseRandomWeightedTriangle(mBody, false);
      else if(m_pointSelection == "eV")
        return ChooseExtremeVertex(mBody, false);
      else if(m_pointSelection == "cM_rV") { 
        if(LRand() % 2)
          return ChooseCenterOfMass(mBody);
        else 
          return ChooseRandomVertex(mBody, false);
      }
      else if(m_pointSelection == "rV_rT") {  
        if(LRand() % 2)
          return ChooseRandomVertex(mBody, false);
        else 
          return ChooseRandomTriangle(mBody, false);
      }
      else if(m_pointSelection == "rV_rW") {
        if(LRand() % 2)
          return ChooseRandomVertex(mBody, false);
        else
          return ChooseRandomWeightedTriangle(mBody, false);
      } 
      else if(m_pointSelection == "all") {
        switch(LRand() % 5) {
          case 0:
            return ChooseCenterOfMass(mBody);
          case 1:
            return ChooseRandomVertex(mBody, false);
          case 2:
            return ChooseRandomTriangle(mBody, false);
          case 3:
            return ChooseRandomWeightedTriangle(mBody, false);
          default:
            return ChooseExtremeVertex(mBody, false);
        }
      } 
      else {
        cerr << "Select a valid point selection type first. Exiting." << endl;
        exit(-1);
      }
    }
};

#endif
