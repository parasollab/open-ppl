#ifndef UNIFORMMEDIALAXISSAMPLER_H_
#define UNIFORMMEDIALAXISSAMPLER_H_

#include "SamplerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

//static bool outDebug = false;

template<typename MPTraits>
class UniformMedialAxisSampler : public SamplerMethod<MPTraits> {
  private:
    double m_length;
    double m_stepSize;
    bool m_useBoundary;
    string m_vcLabel, m_dmLabel;
    ClearanceUtility<MPTraits> m_clearanceUtility;

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    UniformMedialAxisSampler(Environment* _env = NULL,
        string _vcLabel = "", string _dmLabel = "",
        double _length = 0, double _stepSize = 0, bool _useBoundary = false,
        const ClearanceUtility<MPTraits>& _clearanceUtility = ClearanceUtility<MPTraits>())
      : m_length(_length), m_stepSize(_stepSize), m_useBoundary(_useBoundary),
      m_vcLabel(_vcLabel), m_dmLabel(_dmLabel),
      m_clearanceUtility(_clearanceUtility) {
        this->SetName("UniformMedialAxisSampler");
      }

    UniformMedialAxisSampler(MPProblemType* _problem, XMLNodeReader& _node)
      : SamplerMethod<MPTraits>(_problem, _node),
      m_clearanceUtility(_problem, _node) {
        this->SetName("UniformMedialAxisSampler");
        ParseXML(_node);
      }

    void ParseXML(XMLNodeReader& _node) {
      m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
      m_dmLabel =_node.stringXMLParameter("dmLabel", true, "default", "Distance Metric Method");
      m_length = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "generate line segment with length d");
      m_stepSize = _node.numberXMLParameter("t", false, 1, 0, MAX_INT, "the step size t, as a factor of the resolution");
      m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<MPTraits>::PrintOptions(_os);
      m_clearanceUtility.PrintOptions(_os);
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
      _os << "\tlength = " << m_length << endl;
      _os << "\tstepSize = " << m_stepSize << endl;
      _os << "\tuseBoundary = " << m_useBoundary << endl;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb,
        StatClass& _stats, CfgType& _cfgIn,
        vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {

      _stats.IncNodesAttempted(this->GetNameAndLabel());

      ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

      string callee(this->GetNameAndLabel() + "::SampleImpl()");
      CDInfo cdInfo;

      bool generated = false;
      bool cfg1Free;
      int cfg1Witness;

      double length = m_length ? m_length : _env->GetMultiBody(_cfgIn.GetRobotIndex())->GetMaxAxisRange();

      //extend boundary
      _env->ExpandBoundary(length, _cfgIn.GetRobotIndex());

      //Generate first cfg
      CfgType cfg1 = _cfgIn;
      if(cfg1 == CfgType())
        cfg1.GetRandomCfg(_env, _bb);

      //restore boundary
      _env->ExpandBoundary(-length - 2*_env->GetMultiBody(_cfgIn.GetRobotIndex())->GetBoundingSphereRadius(), _cfgIn.GetRobotIndex());

      cfg1Free = vc->IsValid(cfg1, _env, _stats, cdInfo, &callee) && !vc->IsInsideObstacle(cfg1, _env, cdInfo);

      /*VDClearAll();
        VDComment("Cfg1");
        VDAddTempCfg(cfg1, cfg1Free);
        */

      CfgType tmp;
      m_clearanceUtility.CollisionInfo(cfg1, tmp, _bb, cfg1.m_clearanceInfo);
      cfg1Witness = cfg1.m_clearanceInfo.m_nearestObstIndex;

      /*CfgType t1;
        for(size_t i = 0; i<CfgType::DOF(); ++i)
        t1[i] = cfg1.m_clearanceInfo.m_objectPoint[i];
        VDAddTempCfg(t1, false);
        VDClearLastTemp();
        */
      //cout << "Witness Point::" << cfg1.m_clearanceInfo.m_objectPoint << endl;

      CfgType cfg2;
      CfgType incr;

      incr.GetRandomRay(length, _env, dm);
      cfg2 = cfg1 + incr;

      //VDComment("Cfg2");
      //VDAddTempCfg(cfg2, true);

      CfgType tick = cfg1, temp = cfg1;
      int tickWitness, tempWitness = cfg1Witness;

      int nTicks;
      CfgType inter;
      inter.FindIncrement(cfg1, cfg2, &nTicks,
          m_stepSize * _env->GetPositionRes(), m_stepSize * _env->GetOrientationRes());

      for(int i=1; i<nTicks; ++i) {

        tick += inter;
        m_clearanceUtility.CollisionInfo(tick, tmp, _bb, tick.m_clearanceInfo);
        tickWitness = tick.m_clearanceInfo.m_nearestObstIndex;

        /*VDAddTempCfg(tick, true);
          CfgType t1;
          for(size_t i = 0; i<CfgType::DOF(); ++i)
          t1[i] = tick.m_clearanceInfo.m_objectPoint[i];
          VDAddTempCfg(t1, false);
          VDClearLastTemp();
          VDClearLastTemp();
          */
        bool crossed = CheckMedialAxisCrossing(_env, temp, tempWitness, tick, tickWitness);
        if(crossed) {
          //cout << "Crossed, entering binary search." << endl;
          if(BinarySearch(_env, _bb, _stats, temp, tempWitness, tick, tickWitness, _cfgOut)) {
            generated = true;
          }
        }

        tempWitness = tickWitness;
        temp = tick;
      }
      return generated;
    }

  protected:

    bool CheckMedialAxisCrossing(Environment* _env,
        const CfgType& _c1, int _w1,
        const CfgType& _c2, int _w2) {

      //The closest obstacle is the same, check if the triangles which the
      //witness points belong to are adjacent and form a concave face
      if(_w1 == _w2) {
        //Find the triangles which the witness points belong to first
        //assume obstacle multibodies have 1 body
        int tempID = FindVertex(_env, _w1, _c1);
        if(tempID == 1) {
          tempID = FindTriangle(_env, _w1, _c1);
          assert(tempID != -1); //triangle id should be found! error!
        }
        int tickID = FindVertex(_env, _w2, _c2);
        if(tickID == 1) {
          tickID = FindTriangle(_env, _w2, _c2);
          assert(tickID != -1); //triangle id should be found! error!
        }

        if(tempID != tickID) {
          //vertex-vertex
          if(tempID < 0 && tickID < 0) {
            return CheckVertVert(_env, _w1, -(tempID+1), -(tickID+1));
          }
          //tiangle-triangle
          else if(tempID >=0 && tickID >= 0) {
            bool tt = CheckTriTri(_env, _w1, tempID, tickID);
            /*if(outDebug) {
              CfgType t1, t2;
              for(size_t i = 0; i<CfgType::DOF(); ++i) {
                t1[i] = _c1.m_clearanceInfo.m_objectPoint[i];
                t2[i] = _c2.m_clearanceInfo.m_objectPoint[i];
              }
              VDAddTempCfg(_c1, true);
              VDAddTempCfg(t1, false);
              VDAddTempCfg(_c2, true);
              VDAddTempCfg(t2, false);
              VDClearAll();
              outDebug = false;
            }*/
            return tt;
          }
          //triangle-vertex
          else {
            if(tempID < 0)
              return CheckVertTri(_env, _w1, -(tempID+1), tickID);
            else
              return CheckVertTri(_env, _w1, -(tickID+1), tempID);
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

    int FindVertex(Environment* _env, int _witness, const CfgType& _c) {
      StatClass* stat = this->GetMPProblem()->GetStatClass();
      stat->StartClock("FindVertex");
      //Find the vertex which the witness points belong to first
      //assume obstacle multibodies have 1 body
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_witness)->GetBody(0)->GetPolyhedron();
      const Transformation& t = _env->GetMultiBody(_witness)->GetBody(0)->WorldTransformation();

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

    int FindTriangle(Environment* _env, int _witness, const CfgType& _c) {
      StatClass* stat = this->GetMPProblem()->GetStatClass();
      stat->StartClock("FindTriangle");
      //Find the triangles which the witness points belong to first
      //assume obstacle multibodies have 1 body
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_witness)->GetBody(0)->GetPolyhedron();
      const Transformation& t = _env->GetMultiBody(_witness)->GetBody(0)->WorldTransformation();

      Vector3d witnessPoint = -t * _c.m_clearanceInfo.m_objectPoint;

      int id = -1;
      for(size_t i=0; i < polyhedron.m_polygonList.size(); ++i) {

        GMSPolygon& poly = polyhedron.m_polygonList[i];

        //Find the max dimension among the normal vector so we know which
        //two dimensions (except the max dimension) form the surface (in
        //order to call PtInTriangle function)
        double maxv = fabs(poly.m_normal[0]);
        int maxDim = 0;
        if(fabs(poly.m_normal[1]) > maxv) {
          maxv = fabs(poly.m_normal[1]);
          maxDim = 1;
        }
        if(fabs(poly.m_normal[2]) > maxv) {
          maxv = fabs(poly.m_normal[2]);
          maxDim = 2;
        }

        const Vector3d& v0 = polyhedron.m_vertexList[poly.m_vertexList[0]];
        const Vector3d& v1 = polyhedron.m_vertexList[poly.m_vertexList[1]];
        const Vector3d& v2 = polyhedron.m_vertexList[poly.m_vertexList[2]];

        Vector3d vp = witnessPoint - v0;
        double projDist = (witnessPoint - (witnessPoint - (poly.m_normal * (vp*poly.m_normal)))).norm();
        if(projDist <= 0.0001) {
          //p0, p1, p2 are the vertices of the triangle
          size_t dim1 = min((maxDim+1) % 3, (maxDim+2) % 3);
          size_t dim2 = max((maxDim+1) % 3, (maxDim+2) % 3);
          Point2d p0(v0[dim1], v0[dim2]);
          Point2d p1(v1[dim1], v1[dim2]);
          Point2d p2(v2[dim1], v2[dim2]);
          Point2d ptemp(witnessPoint[dim1], witnessPoint[dim2]);

          //Store the triangle id if the witness point belongs to it
          /*cout << "\n::PointInTriangle Test::" << i << endl
            << "v0:: " << v0 << "\tv1:: " << v1 << "\tv2:: " << v2 << endl
            << "WitnessPoint:: " << witnessPoint << endl
            << "Normal:: " << poly.m_normal << endl
            << "p0:: " << p0 << "\tp1:: " << p1 << "\tp2:: " << p2 << endl
            << "ptemp:: " << ptemp << endl
            << "ptempIn:: " << PtInTriangle(p0, p1, p2, ptemp) << endl;
            */
          double u,v;
          bool inTri = PtInTriangle(p0, p1, p2, ptemp, u, v);
          //cout << "u::" << u << "\tv::" << v << endl;
          if(inTri) {
            //cout << "projDist::" << projDist << endl;
            stat->StopClock("FindTriangle");
            return i;
            //id = i;
            //minProjDist = projDist;
          }
        }
      }

      //if the triangle IDs have not been set, there has been an error in
      //the triangle computation
      stat->StopClock("FindTriangle");
      return id;
    }

    bool CheckVertVert(Environment* _env, int _w, int _v1, int _v2) {
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_w)->GetBody(0)->GetPolyhedron();
      vector<GMSPolygon>& polygons = polyhedron.m_polygonList;

      typedef vector<GMSPolygon>::iterator PIT;
      for(PIT pit = polygons.begin(); pit!=polygons.end(); ++pit) {
        vector<int>& verts = pit->m_vertexList;
        if(find(verts.begin(), verts.end(), _v1) != verts.end() &&
            find(verts.begin(), verts.end(), _v2) != verts.end())
          return false;
      }

      return true;
    }

    bool CheckTriTri(Environment* _env, int _w, int _t1, int _t2) {
      //Check if two triangles are adjacent to each other
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_w)->GetBody(0)->GetPolyhedron();

      //test if there is a common edge (v0, v1) between the triangles
      pair<int, int> edge = polyhedron.m_polygonList[_t1].CommonEdge(polyhedron.m_polygonList[_t2]);
      if(edge.first != -1 && edge.second != -1) {
        //Get vertex information for two facets
        const Vector3d& v0 = polyhedron.m_vertexList[edge.first];
        const Vector3d& v1 = polyhedron.m_vertexList[edge.second];
        Vector3d v2;
        for(vector<int>::iterator I = polyhedron.m_polygonList[_t1].m_vertexList.begin(); I != polyhedron.m_polygonList[_t1].m_vertexList.end(); I++) {
          if((*I != edge.first) && (*I != edge.second))
            v2 = polyhedron.m_vertexList[*I];
        }
        //Find out which triangle is on the left and which is on the right
        Vector3d va = v1 - v0;  //Common edge (v0, v1)
        Vector3d vb = v2 - v0;  //The other edge (v0, v2)
        int left, right;
        //The face is on the left of the common edge if (va x vb).normal vector of the face > 0
        //If < 0, the face is on the right
        if(((va % vb) * polyhedron.m_polygonList[_t1].m_normal) > 0) {
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
        if(((polyhedron.m_polygonList[left].m_normal % polyhedron.m_polygonList[right].m_normal) * va) < -0.0001){  //Concave
          return true;
        }
        else {  //Convex
          return false;
        }
      }

      //test if there is one common vertex between the triangles
      int vert = polyhedron.m_polygonList[_t1].CommonVertex(polyhedron.m_polygonList[_t2]);
      if(vert != -1) {
        return !_env->GetMultiBody(_w)->GetBody(0)->IsConvexHullVertex(polyhedron.m_vertexList[vert]);
        //cout << "There is only one common vertex it seems -_-" << endl;
        //Vector3d& n1 = polyhedron.m_polygonList[_t1].m_normal;
        //Vector3d& n2 = polyhedron.m_polygonList[_t2].m_normal;
        //return asin((n1 % n2).norm()/(n1.norm() * n2.norm())) > 0;
      }
      //no common edge or vertex, triangles are not adjacent
      else {
        //outDebug = true;
        return true;
      }
    }

    bool CheckVertTri(Environment* _env, int _w, int _v, int _t) {
      //Check if vertex belongs to triangle, thus are adjacent
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_w)->GetBody(0)->GetPolyhedron();
      Vector3d& vert = polyhedron.m_vertexList[_v];
      GMSPolygon& poly = polyhedron.m_polygonList[_t];
      for(size_t i = 0; i<poly.m_vertexList.size(); ++i) {
        //shares a common vertex, return false!
        if(vert == polyhedron.m_vertexList[poly.m_vertexList[i]])
          return false;
      }
      //vertex and triangle are separate, must've crossed the medial axis
      return true;
    }

    bool BinarySearch(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats,
        const CfgType& _c1, int _w1, const CfgType& _c2, int _w2,
        vector<CfgType>& _cfgOut) {

      CfgType left = _c1, right = _c2;
      //grab initial IDs
      int leftOI = _w1, rightOI = _w2;
      int leftID = leftOI, rightID = rightOI;
      if(_w1 == _w2) {
        leftID = FindVertex(_env, leftOI, left);
        if(leftID == 1) {
          leftID = FindTriangle(_env, leftOI, left);
          assert(leftID != -1); //triangle id should be found! error!
        }
        rightID = FindVertex(_env, rightOI, right);
        if(rightID == 1) {
          rightID = FindTriangle(_env, rightOI, right);
          assert(rightID != -1); //triangle id should be found! error!
        }
      }

      //iterate until distance between _c1 and _c2 is below
      //environment resolution
      CfgType incr;
      int nTicks;
      incr.FindIncrement(left, right, &nTicks, _env->GetPositionRes(), _env->GetOrientationRes());

      while(nTicks > 1) {
        //compute midpoint
        CfgType mid = (left + right) / 2;

        //grab witness id
        CfgType tmp;
        m_clearanceUtility.CollisionInfo(mid, tmp, _bb, mid.m_clearanceInfo);
        int midOI = mid.m_clearanceInfo.m_nearestObstIndex;

        //discovered new obstacle index, need to recurse on each side
        if(midOI != leftOI || midOI != rightOI) {
          bool l = false;
          if(CheckMedialAxisCrossing(_env, left, leftOI, mid, midOI))
            l = BinarySearch(_env, _bb, _stats, left, leftOI, mid, midOI, _cfgOut);
          bool r = false;
          if(CheckMedialAxisCrossing(_env, mid, midOI, right, rightOI))
            r = BinarySearch(_env, _bb, _stats, mid, midOI, right, rightOI, _cfgOut);
          if(l || r)
            return true;
          else
            return false;
        }

        //find triangle id
        int midID = midOI;
        if(_w1 == _w2) {
          midID = FindVertex(_env, midOI, mid);
          if(midID == 1) {
            midID = FindTriangle(_env, midOI, mid);
            assert(midID != -1); //triangle id should be found! error!
          }
        }

        //search on right half
        if(midID == leftID) {
          //left = mid
          leftOI = midOI;
          leftID = midID;
          left = mid;
        }
        //search on left half
        else if(midID == rightID) {
          //right = mid
          rightOI = midOI;
          rightID = midID;
          right = mid;
        }
        //middle has completely different midpoint, recurse on each half
        else {
          bool l = false;
          if(CheckMedialAxisCrossing(_env, left, leftOI, mid, midOI))
            l = BinarySearch(_env, _bb, _stats, left, leftOI, mid, midOI, _cfgOut);
          bool r = false;
          if(CheckMedialAxisCrossing(_env, mid, midOI, right, rightOI))
            r = BinarySearch(_env, _bb, _stats, mid, midOI, right, rightOI, _cfgOut);
          if(l || r)
            return true;
          else
            return false;
        }

        //set nticks
        incr.FindIncrement(left, right, &nTicks, _env->GetPositionRes(), _env->GetOrientationRes());
      }

      //keep witness with higher clearance
      if(left.m_clearanceInfo.m_minDist > 0 || right.m_clearanceInfo.m_minDist > 0) {
        const CfgType& higher = left.m_clearanceInfo.m_minDist > right.m_clearanceInfo.m_minDist ? left : right;

        if(_env->InBounds(higher, _bb)) {
          _stats.IncNodesGenerated(this->GetNameAndLabel());
          _cfgOut.push_back(higher);
          return true;
        }
      }
      return false;
    }
};

#endif

