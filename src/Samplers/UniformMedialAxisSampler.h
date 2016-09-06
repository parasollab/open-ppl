#ifndef UNIFORM_MEDIAL_AXIS_SAMPLER_H_
#define UNIFORM_MEDIAL_AXIS_SAMPLER_H_

#include "SamplerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief TODO
/// @tparam MPTraits Motion planning universe
///
/// TODO
////////////////////////////////////////////////////////////////////////////////
template<typename MPTraits>
class UniformMedialAxisSampler : public SamplerMethod<MPTraits> {

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    UniformMedialAxisSampler(string _vcLabel = "", string _dmLabel = "",
        double _length = 0, double _stepSize = 0, bool _useBoundary = false,
        const ClearanceUtility<MPTraits>& _clearanceUtility = ClearanceUtility<MPTraits>());

    UniformMedialAxisSampler(MPProblemType* _problem, XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision);

  protected:

    bool CheckMedialAxisCrossing(const CfgType& _c1, int _w1,
        const CfgType& _c2, int _w2);

    int FindVertex(int _witness, const CfgType& _c);
    int FindTriangle(int _witness, const CfgType& _c);
    bool CheckVertVert(int _w, int _v1, int _v2);
    bool CheckTriTri(int _w, int _t1, int _t2);
    bool CheckVertTri(int _w, int _v, int _t);

    bool BinarySearch(shared_ptr<Boundary> _boundary,
        const CfgType& _c1, int _w1, const CfgType& _c2, int _w2,
        vector<CfgType>& _result);

  private:
    double m_length;
    double m_stepSize;
    bool m_useBoundary;
    string m_vcLabel, m_dmLabel;
    ClearanceUtility<MPTraits> m_clearanceUtility;
};

template<class MPTraits>
UniformMedialAxisSampler<MPTraits>::
UniformMedialAxisSampler(string _vcLabel, string _dmLabel,
    double _length, double _stepSize, bool _useBoundary,
    const ClearanceUtility<MPTraits>& _clearanceUtility) :
  m_length(_length), m_stepSize(_stepSize), m_useBoundary(_useBoundary),
  m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
  m_clearanceUtility(_clearanceUtility) {
    this->SetName("UniformMedialAxisSampler");
  }

template<class MPTraits>
UniformMedialAxisSampler<MPTraits>::
UniformMedialAxisSampler(MPProblemType* _problem, XMLNode& _node)
  : SamplerMethod<MPTraits>(_problem, _node),
  m_clearanceUtility(_problem, _node) {
    this->SetName("UniformMedialAxisSampler");
    ParseXML(_node);
  }

template<class MPTraits>
void
UniformMedialAxisSampler<MPTraits>::
ParseXML(XMLNode& _node) {
  m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
  m_dmLabel =_node.Read("dmLabel", true, "default", "Distance Metric Method");
  m_length = _node.Read("d", true, 0.0, 0.0, MAX_DBL,
      "generate line segment with length d");
  m_stepSize = _node.Read("t", false, 1, 0, MAX_INT,
      "the step size t, as a factor of the resolution");
  m_useBoundary = _node.Read("useBBX", true, false,
      "Use bounding box as obstacle");
}

template<class MPTraits>
void
UniformMedialAxisSampler<MPTraits>::
Print(ostream& _os) const {
  SamplerMethod<MPTraits>::Print(_os);
  m_clearanceUtility.Print(_os);
  _os << "\tvcLabel = " << m_vcLabel << endl;
  _os << "\tdmLabel = " << m_dmLabel << endl;
  _os << "\tlength = " << m_length << endl;
  _os << "\tstepSize = " << m_stepSize << endl;
  _os << "\tuseBoundary = " << m_useBoundary << endl;
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
    vector<CfgType>& _result, vector<CfgType>& _collision) {

  Environment* env = this->GetEnvironment();
  ValidityCheckerPointer vc = this->GetValidityChecker(m_vcLabel);
  DistanceMetricPointer dm = this->GetDistanceMetric(m_dmLabel);

  string callee(this->GetNameAndLabel() + "::SampleImpl()");
  CDInfo cdInfo;

  bool generated = false;
  int cfg1Witness;

  double length = m_length ? m_length :
    env->GetRobot(_cfg.GetRobotIndex())->GetMaxAxisRange();

  //extend boundary
  env->ExpandBoundary(length, _cfg.GetRobotIndex());

  //Generate first cfg
  CfgType& cfg1 = _cfg;

  //restore boundary
  env->ExpandBoundary(-length - 2*env->GetRobot(_cfg.GetRobotIndex())->
      GetBoundingSphereRadius(), _cfg.GetRobotIndex());

  CfgType tmp;
  m_clearanceUtility.CollisionInfo(cfg1, tmp, _boundary, cfg1.m_clearanceInfo);
  cfg1Witness = cfg1.m_clearanceInfo.m_nearestObstIndex;

  CfgType cfg2;
  CfgType incr;

  incr.GetRandomRay(length, dm);
  cfg2 = cfg1 + incr;

  CfgType tick = cfg1, temp = cfg1;
  int tickWitness, tempWitness = cfg1Witness;

  int nTicks;
  CfgType inter;
  inter.FindIncrement(cfg1, cfg2, &nTicks,
      m_stepSize * env->GetPositionRes(),
      m_stepSize * env->GetOrientationRes());

  for(int i = 1; i < nTicks; ++i) {

    tick += inter;
    m_clearanceUtility.CollisionInfo(tick, tmp, _boundary, tick.m_clearanceInfo);
    tickWitness = tick.m_clearanceInfo.m_nearestObstIndex;

    bool crossed = CheckMedialAxisCrossing(temp, tempWitness,
        tick, tickWitness);
    if(crossed) {
      if(BinarySearch(_boundary, temp, tempWitness, tick, tickWitness, _result)) {
        generated = true;
      }
    }

    tempWitness = tickWitness;
    temp = tick;
  }

  return generated;
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
CheckMedialAxisCrossing(const CfgType& _c1, int _w1,
    const CfgType& _c2, int _w2) {

  Environment* env = this->GetEnvironment();
  //The closest obstacle is the same, check if the triangles which the
  //witness points belong to are adjacent and form a concave face
  if(_w1 == _w2) {
    //Check if the witness points are on the bounding box
    if(_w1 == -1) {
      int side1 = env->GetBoundary()->GetSideID(_c1.GetData());
      int side2 = env->GetBoundary()->GetSideID(_c2.GetData());
      if(side1 == side2)
        return false;
      else
        return true;
    }

    //Find the triangles which the witness points belong to first
    //assume obstacle multibodies have 1 body
    int tempID = FindVertex(_w1, _c1);
    if(tempID == 1) {
      tempID = FindTriangle(_w1, _c1);
      assert(tempID != -1); //triangle id should be found! error!
    }
    int tickID = FindVertex(_w2, _c2);
    if(tickID == 1) {
      tickID = FindTriangle(_w2, _c2);
      assert(tickID != -1); //triangle id should be found! error!
    }

    if(tempID != tickID) {
      //vertex-vertex
      if(tempID < 0 && tickID < 0) {
        return CheckVertVert(_w1, -(tempID+1), -(tickID+1));
      }
      //tiangle-triangle
      else if(tempID >= 0 && tickID >= 0) {
        bool tt = CheckTriTri(_w1, tempID, tickID);
        return tt;
      }
      //triangle-vertex
      else {
        if(tempID < 0)
          return CheckVertTri(_w1, -(tempID+1), tickID);
        else
          return CheckVertTri(_w1, -(tickID+1), tempID);
      }
    }
    //triangle id didn't change - no medial axis crossing
    else {
      return false;
    }
  }
  //tempWitness != tickWitness, closest obstacle changed!
  else {
    return true;
  }
}

template<class MPTraits>
int
UniformMedialAxisSampler<MPTraits>::
FindVertex(int _witness, const CfgType& _c) {
  Environment* env = this->GetEnvironment();
  StatClass* stat = this->GetStatClass();
  stat->StartClock("FindVertex");
  //Find the vertex which the witness points belong to first
  //assume obstacle multibodies have 1 body
  GMSPolyhedron& polyhedron = env->GetObstacle(_witness)->
    GetFixedBody(0)->GetPolyhedron();
  const Transformation& t = env->GetObstacle(_witness)->
    GetFixedBody(0)->WorldTransformation();

  Vector3d witnessPoint = -t * _c.m_clearanceInfo.m_objectPoint;
  for(size_t i=0; i < polyhedron.m_vertexList.size(); ++i) {
    Vector3d& vert = polyhedron.m_vertexList[i];

    if(witnessPoint == vert) {
      stat->StopClock("FindVertex");
      return -i - 1;
    }
  }
  stat->StopClock("FindVertex");
  return 1;
}

template<class MPTraits>
int
UniformMedialAxisSampler<MPTraits>::
FindTriangle(int _witness, const CfgType& _c) {
  Environment* env = this->GetEnvironment();
  StatClass* stat = this->GetStatClass();
  stat->StartClock("FindTriangle");
  //Find the triangles which the witness points belong to first
  //assume obstacle multibodies have 1 body
  GMSPolyhedron& polyhedron = env->GetObstacle(_witness)->
    GetFixedBody(0)->GetPolyhedron();
  const Transformation& t = env->GetObstacle(_witness)->
    GetFixedBody(0)->WorldTransformation();

  Vector3d witnessPoint = -t * _c.m_clearanceInfo.m_objectPoint;

  int id = -1;
  for(size_t i=0; i < polyhedron.m_polygonList.size(); ++i) {

    GMSPolygon& poly = polyhedron.m_polygonList[i];
    const Vector3d& normal = poly.GetNormal();

    //Find the max dimension among the normal vector so we know which
    //two dimensions (except the max dimension) form the surface (in
    //order to call PtInTriangle function)
    double maxv = fabs(normal[0]);
    int maxDim = 0;
    if(fabs(normal[1]) > maxv) {
      maxv = fabs(normal[1]);
      maxDim = 1;
    }
    if(fabs(normal[2]) > maxv) {
      maxv = fabs(normal[2]);
      maxDim = 2;
    }

    const Vector3d& v0 = poly.GetPoint(0);
    const Vector3d& v1 = poly.GetPoint(1);
    const Vector3d& v2 = poly.GetPoint(2);

    Vector3d vp = witnessPoint - v0;
    double projDist = (witnessPoint - (witnessPoint - (normal *
            (vp * normal)))).norm();
    if(projDist <= 0.0001) {
      //p0, p1, p2 are the vertices of the triangle
      size_t dim1 = min((maxDim+1) % 3, (maxDim+2) % 3);
      size_t dim2 = max((maxDim+1) % 3, (maxDim+2) % 3);
      Point2d p0(v0[dim1], v0[dim2]);
      Point2d p1(v1[dim1], v1[dim2]);
      Point2d p2(v2[dim1], v2[dim2]);
      Point2d ptemp(witnessPoint[dim1], witnessPoint[dim2]);

      //Store the triangle id if the witness point belongs to it
      double u,v;
      bool inTri = PtInTriangle(p0, p1, p2, ptemp, u, v);
      if(inTri) {
        stat->StopClock("FindTriangle");
        return i;
      }
    }
  }

  //if the triangle IDs have not been set, there has been an error in
  //the triangle computation
  stat->StopClock("FindTriangle");
  return id;
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
CheckVertVert(int _w, int _v1, int _v2) {
  Environment* env = this->GetEnvironment();
  GMSPolyhedron& polyhedron = env->GetObstacle(_w)->GetFixedBody(0)->
      GetPolyhedron();
  vector<GMSPolygon>& polygons = polyhedron.m_polygonList;

  typedef vector<GMSPolygon>::iterator PIT;
  for(PIT pit = polygons.begin(); pit!=polygons.end(); ++pit) {
    if(find(pit->begin(), pit->end(), _v1) != pit->end() &&
        find(pit->begin(), pit->end(), _v2) != pit->end())
      return false;
  }

  return true;
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
CheckTriTri(int _w, int _t1, int _t2) {
  Environment* env = this->GetEnvironment();
  //Check if two triangles are adjacent to each other
  GMSPolyhedron& polyhedron = env->GetObstacle(_w)->
    GetFixedBody(0)->GetPolyhedron();

  //test if there is a common edge (v0, v1) between the triangles
  pair<int, int> edge = polyhedron.m_polygonList[_t1].
    CommonEdge(polyhedron.m_polygonList[_t2]);
  if(edge.first != -1 && edge.second != -1) {
    //Get vertex information for two facets
    const Vector3d& v0 = polyhedron.m_vertexList[edge.first];
    const Vector3d& v1 = polyhedron.m_vertexList[edge.second];
    Vector3d v2;
    for(vector<int>::iterator I = polyhedron.m_polygonList[_t1].begin();
        I != polyhedron.m_polygonList[_t1].end(); I++) {
      if((*I != edge.first) && (*I != edge.second))
        v2 = polyhedron.m_vertexList[*I];
    }
    //Find out which triangle is on the left and which is on the right
    Vector3d va = v1 - v0;  //Common edge (v0, v1)
    Vector3d vb = v2 - v0;  //The other edge (v0, v2)
    int left, right;
    //The face is on the left of the common edge if (va x vb).normal
    //vector of the face > 0
    //If < 0, the face is on the right
    if(((va % vb) * polyhedron.m_polygonList[_t1].GetNormal()) > 0) {
      left = _t1;
      right = _t2;
    }
    else {
      left = _t2;
      right = _t1;
    }
    //Check if the two triangles form a concave face
    //If (normal vector of left) x (normal vector of right) is the
    //opposite direction of the common edge, they form a concave face
    if(((polyhedron.m_polygonList[left].GetNormal() % polyhedron.
            m_polygonList[right].GetNormal()) * va) < -0.0001) {  //Concave
      return true;
    }
    else {  //Convex
      return false;
    }
  }

  //test if there is one common vertex between the triangles
  int vert = polyhedron.m_polygonList[_t1].
    CommonVertex(polyhedron.m_polygonList[_t2]);
  if(vert != -1) {
    return !env->GetObstacle(_w)->GetFixedBody(0)->
      IsConvexHullVertex(polyhedron.m_vertexList[vert]);
  }
  //no common edge or vertex, triangles are not adjacent
  else {
    return true;
  }
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
CheckVertTri(int _w, int _v, int _t) {
  Environment* env = this->GetEnvironment();
  //Check if vertex belongs to triangle, thus are adjacent
  GMSPolyhedron& polyhedron = env->GetObstacle(_w)->
    GetFixedBody(0)->GetPolyhedron();
  Vector3d& vert = polyhedron.m_vertexList[_v];
  GMSPolygon& poly = polyhedron.m_polygonList[_t];
  for(size_t i = 0; i < poly.GetNumVertices(); ++i) {
    //shares a common vertex, return false!
    if(vert == poly.GetPoint(i))
      return false;
  }
  //vertex and triangle are separate, must've crossed the medial axis
  return true;
}

template<class MPTraits>
bool
UniformMedialAxisSampler<MPTraits>::
BinarySearch(shared_ptr<Boundary> _boundary,
    const CfgType& _c1, int _w1, const CfgType& _c2, int _w2,
    vector<CfgType>& _result) {

  Environment* env = this->GetEnvironment();
  CfgType left = _c1, right = _c2;
  //grab initial IDs
  int leftOI = _w1, rightOI = _w2;
  int leftID = leftOI, rightID = rightOI;
  if(_w1 == -1)  //the witness point is on the bounding box
    leftID = _boundary->GetSideID(_c1.GetData());
  if(_w2 == -1)  //the witness point is on the bounding box
    rightID = _boundary->GetSideID(_c2.GetData());
  if((_w1 == _w2) && (_w1 != -1)) {
    leftID = FindVertex(leftOI, left);
    if(leftID == 1) {
      leftID = FindTriangle(leftOI, left);
      assert(leftID != -1); //triangle id should be found! error!
    }
    rightID = FindVertex(rightOI, right);
    if(rightID == 1) {
      rightID = FindTriangle(rightOI, right);
      assert(rightID != -1); //triangle id should be found! error!
    }
  }

  //iterate until distance between _c1 and _c2 is below
  //environment resolution
  CfgType incr;
  int nTicks;
  incr.FindIncrement(left, right, &nTicks,
      env->GetPositionRes(), env->GetOrientationRes());

  while(nTicks > 1) {
    //compute midpoint
    CfgType mid = (left + right) / 2;

    //grab witness id
    CfgType tmp;
    m_clearanceUtility.CollisionInfo(mid, tmp, _boundary, mid.m_clearanceInfo);
    int midOI = mid.m_clearanceInfo.m_nearestObstIndex;

    //discovered new obstacle index, need to recurse on each side
    if(midOI != leftOI && midOI != rightOI) {
      bool l = false;
      if(CheckMedialAxisCrossing(left, leftOI, mid, midOI))
        l = BinarySearch(_boundary, left, leftOI, mid, midOI, _result);
      bool r = false;
      if(CheckMedialAxisCrossing(mid, midOI, right, rightOI))
        r = BinarySearch(_boundary, mid, midOI, right, rightOI, _result);
      if(l || r)
        return true;
      else
        return false;
    }

    //find triangle id
    int midID = midOI;
    if(midOI == -1)  //witness point is on the bounding box
      midID = _boundary->GetSideID(mid.GetData());
    if((_w1 == _w2) && (_w1 != -1)) {
      midID = FindVertex(midOI, mid);
      if(midID == 1) {
        midID = FindTriangle(midOI, mid);
        assert(midID != -1); //triangle id should be found! error!
      }
    }

    //search on right half
    if(midID == leftID) {
      leftOI = midOI;
      leftID = midID;
      left = mid;
    }
    //search on left half
    else if(midID == rightID) {
      rightOI = midOI;
      rightID = midID;
      right = mid;
    }
    //middle has completely different midpoint, recurse on each half
    else {
      bool l = false;
      if(CheckMedialAxisCrossing(left, leftOI, mid, midOI))
        l = BinarySearch(_boundary, left, leftOI, mid, midOI, _result);
      bool r = false;
      if(CheckMedialAxisCrossing(mid, midOI, right, rightOI))
        r = BinarySearch(_boundary, mid, midOI, right, rightOI, _result);
      if(l || r)
        return true;
      else
        return false;
    }

    //set nticks
    incr.FindIncrement(left, right, &nTicks,
        env->GetPositionRes(), env->GetOrientationRes());
  }

  //keep witness with higher clearance
  if(left.m_clearanceInfo.m_minDist > 0 || right.m_clearanceInfo.m_minDist > 0) {
    CfgType& higher = left.m_clearanceInfo.m_minDist >
      right.m_clearanceInfo.m_minDist ? left : right;

    ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
    CDInfo cdInfo;
    string callee = this->GetNameAndLabel() + "::BinS";
    bool cfgFree = env->InBounds(higher)
      && vc->IsValid(higher, cdInfo, callee)
      && !vc->IsInsideObstacle(higher);

    if(cfgFree) {
      _result.push_back(higher);
      return true;
    }
  }
  return false;
}

#endif

