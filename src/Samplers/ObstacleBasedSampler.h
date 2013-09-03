// OBPRM samples by pushing random configurations along a random ray
// until they change validity, keeping the best free configuration

#ifndef OBSTACLEBASEDSAMPLER_H_
#define OBSTACLEBASEDSAMPLER_H_

#include "SamplerMethod.h"

class Environment;
class StatClass;
class CDInfo;

template <typename MPTraits>
class ObstacleBasedSampler : public SamplerMethod<MPTraits> {
  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    ObstacleBasedSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "",
        int _free = 1, int _coll = 0, double _step = 0, bool _useBBX = true, string _pointSelection = "cspace")
      : m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_nShellsFree(_free), m_nShellsColl(_coll), m_stepSize(_step), m_useBBX(_useBBX), m_pointSelection(_pointSelection) {
        this->SetName("ObstacleBasedSampler");
        // If the step size is unreasonable, set it to the minimum
        if(m_stepSize <= 0.0) {
          if(_env)
            m_stepSize = min(_env->GetPositionRes(), _env->GetOrientationRes());
        }
      }

    ObstacleBasedSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node) {
      this->SetName("ObstacleBasedSampler");
      ParseXML(_node);
      Environment* env = _problem->GetEnvironment();
      // If the step size is unreasonable, set it to the minimum
      if(m_stepSize <= 0.0)
        m_stepSize = min(env->GetPositionRes(), env->GetOrientationRes());
    }

    ~ObstacleBasedSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_useBBX = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
      m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity test method");
      m_dmLabel =_node.stringXMLParameter("dmLabel", true, "default", "Distance metric method");
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
      SamplerMethod<MPTraits>::PrintOptions(_os);
      _os << "\tnShellsFree = " << m_nShellsFree << endl;
      _os << "\tnShellsColl = " << m_nShellsColl << endl;
      _os << "\tstepSize = " << m_stepSize << endl;
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
      _os << "\tuseBBX = " << m_useBBX << endl;
      _os << "\tpointSelectionStrategy = " << m_pointSelection << endl;
    }

    // Generates and adds shells to their containers
    template<typename OutputIterator>
      OutputIterator GenerateShells(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
          CfgType& _cFree, CfgType& _cColl, CfgType& _incr, OutputIterator _result) {

        string callee = this->GetNameAndLabel() + "::GenerateShells()";
        ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
        CDInfo cdInfo;
        if(this->m_debug)
          cout << "nShellsColl = " << m_nShellsColl << endl;

        // Add free shells
        for(int i = 0; i < m_nShellsFree; i++) {
          // If the shell is valid
          if(_env->InBounds(_cFree, _bb) &&
              vcm->IsValid(_cFree, _env, _stats, cdInfo, callee)) {
            if(this->m_recordKeep)
              _stats.IncNodesGenerated(this->GetNameAndLabel());
            // Add shell
            *_result = _cFree;
            _result++;
          }
          // Get next shell
          _cFree += _incr;
        }

        // Reverse direction of _incr
        _incr = -_incr;

        // Add collision shells
        for(int i = 0; i < m_nShellsColl; i++) {
          // If the shell is valid
          if(_env->InBounds(_cColl, _bb) &&
              !vcm->IsValid(_cColl, _env, _stats, cdInfo, callee)) {
            if(this->m_recordKeep)
              _stats.IncNodesGenerated(this->GetNameAndLabel());
            // Add shell
            *_result = _cColl;
            _result++;
          }
          // Get next shell
          _cColl += _incr;
        }
        return _result;
      }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
        CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {

      string callee = this->GetNameAndLabel() + "::Sampler()";
      ValidityCheckerPointer vcm = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);
      CDInfo cdInfo;

      if(this->m_recordKeep)
        _stats.IncNodesAttempted(this->GetNameAndLabel());

      // Old state
      CfgType c1 = ChooseASample(_cfgIn, _env, _bb);
      bool c1BBox = _env->InBounds(c1, _bb);
      bool c1Free = vcm->IsValid(c1, _env, _stats, cdInfo, callee);

      // New state
      CfgType c2 = c1;
      bool c2BBox = c1BBox;
      bool c2Free = c1Free;

      // Create a random ray
      CfgType r;
      r.GetRandomRay(m_stepSize, _env, dm);
      if(r == CfgType()) {
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
        c2 += r;
        c2BBox = _env->InBounds(c2, _bb);
        c2Free = vcm->IsValid(c2, _env, _stats, cdInfo, callee);
      }

      // If new state is in BBox (there must be a validity change)
      if(c2BBox) {
        if(c1Free) { // Old state (c1) is free
          // Reverse direction of r
          r = -r;
          // Process configurations
          GenerateShells(_env, _bb, _stats, c1, c2, r, back_insert_iterator<vector<CfgType> >(_cfgOut));
          _cfgCol.push_back(c2);
        }
        else { // New state (c2) is free
          // Process configurations
          GenerateShells(_env, _bb, _stats, c2, c1, r,back_insert_iterator<vector<CfgType> >(_cfgOut));
          _cfgCol.push_back(c1);
        }
        return true;
      }
      else if(m_useBBX && c1BBox && c1Free) {
        // Reverse direction of r
        r = -r;
        // Process configurations
        GenerateShells(_env, _bb, _stats, c1, c2, r, back_insert_iterator<vector<CfgType> >(_cfgOut));
        _cfgCol.push_back(c2);
        return true;
      }
      return false;
    }

    ////////////////////////////////////////////////////////////////
    // The following was added after unification with WOBPRM code //
    ////////////////////////////////////////////////////////////////

    // Returns a CfgType at the center of mass of the MultiBody
    CfgType ChooseCenterOfMass(shared_ptr<MultiBody> _mBody) {
      Vector3d x = _mBody->GetCenterOfMass();
      return GetCfgWithParams(x);
    }

    // Returns a CfgType at a random vertex of the MultiBody
    CfgType ChooseRandomVertex(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      Vector3d x = polyhedron.m_vertexList[(int)(DRand()*polyhedron.m_vertexList.size())];
      return GetCfgWithParams(x);
    }

    // Returns a point inside the triangle determined by the vectors
    Vector3d ChoosePointOnTriangle(Vector3d _p, Vector3d _q, Vector3d _r) {
      Vector3d u = _q - _p; // From _p to _q
      Vector3d v = _r - _p; // From _p to _r
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
    CfgType ChooseRandomWeightedTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
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
      Vector3d p = polyhedron.m_vertexList[poly->m_vertexList[0]];
      Vector3d q = polyhedron.m_vertexList[poly->m_vertexList[1]];
      Vector3d r = polyhedron.m_vertexList[poly->m_vertexList[2]];
      Vector3d x = ChoosePointOnTriangle(p, q, r);
      return GetCfgWithParams(x);
    }

    // Chooses a random point in a random triangle in the MultiBody
    CfgType ChooseRandomTriangle(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
      GMSPolyhedron polyhedron = GetPolyhedron(_mBody, _isFreeBody);
      // Choose a random triangle
      GMSPolygon *poly = &polyhedron.m_polygonList[(int)(DRand()*polyhedron.m_polygonList.size())];
      Vector3d p = polyhedron.m_vertexList[poly->m_vertexList[0]];
      Vector3d q = polyhedron.m_vertexList[poly->m_vertexList[1]];
      Vector3d r = polyhedron.m_vertexList[poly->m_vertexList[2]];
      Vector3d x = ChoosePointOnTriangle(p, q, r);
      return GetCfgWithParams(x);
    }

    // Chooses a random extreme vertex of the MultiBody
    CfgType ChooseExtremeVertex(shared_ptr<MultiBody> _mBody, bool _isFreeBody) {
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

    // Checks m_pointSelection and returns an appropriate CfgType
    virtual CfgType ChooseASample(CfgType& _cfgIn, Environment* _env, shared_ptr<Boundary> _bb) {
      shared_ptr<MultiBody> mBody;
      if(m_pointSelection != "cspace") {
        mBody = _env->GetRandomObstacle();
      }
      // cspace is for Configuration space (This is for unifying OBPRM and WOBPRM)
      if(m_pointSelection == "cspace") {
        if(_cfgIn == CfgType())
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

  private:

    // Returns a CfgType with the coordinates specified in the vector and no rotation
    CfgType GetCfgWithParams(Vector3d& _v) {
      CfgType tmp;
      for(int i = 0; i < 3; i++)
        tmp[i] = _v[i];
      for(int i = 3; i < 6; i++)
        tmp[i] = 0.0;
      return tmp;
    }

    // Returns an appropriate polygon: for a robot, the _mBody frame; for an obstacle, the world frame
    GMSPolyhedron GetPolyhedron(shared_ptr<MultiBody>& _mBody, bool _isFreeBody) {
      if(_isFreeBody)
        return _mBody->GetBody(0)->GetPolyhedron();
      else
        return _mBody->GetBody(0)->GetWorldPolyhedron();
    }

    string m_vcLabel, m_dmLabel; // Validity checker method, distance metric method
    int m_nShellsFree, m_nShellsColl; // Number of free and collision shells
    double m_stepSize; // Step size along the random ray
    bool m_useBBX; // Is the bounding box an obstacle?
    string m_pointSelection; // Needed for the WOBPRM
};

#endif
