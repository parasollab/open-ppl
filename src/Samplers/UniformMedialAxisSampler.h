#ifndef UNIFORMMEDIALAXISSAMPLER_H_
#define UNIFORMMEDIALAXISSAMPLER_H_

#include "SamplerMethod.h"
#include "Utilities/MedialAxisUtilities.h"

template<typename MPTraits>
class UniformMedialAxisSampler : public SamplerMethod<MPTraits> {
  private:
    double m_length;
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
        double _length = 0, bool _useBoundary = false,
        const ClearanceUtility<MPTraits>& _clearanceUtility = ClearanceUtility<MPTraits>())
      : m_length(_length), m_useBoundary(_useBoundary),
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
      m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<MPTraits>::PrintOptions(_os);
      m_clearanceUtility.PrintOptions(_os);
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
      _os << "\tlength = " << m_length << endl;
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

      //Generate first cfg
      CfgType cfg1 = _cfgIn;
      if(cfg1 == CfgType())
        cfg1.GetRandomCfg(_env, _bb);

      cfg1Free = vc->IsValid(cfg1, _env, _stats, cdInfo, &callee) && !vc->IsInsideObstacle(cfg1, _env, cdInfo);

      VDClearAll();
      VDComment("Cfg1");
      VDAddTempCfg(cfg1, cfg1Free);

      CfgType tmp;
      m_clearanceUtility.CollisionInfo(cfg1, tmp, _bb, cfg1.m_clearanceInfo);
      cfg1Witness = cfg1.m_clearanceInfo.m_nearestObstIndex;

      CfgType t1;
      for(size_t i = 0; i<CfgType::DOF(); ++i)
        t1[i] = cfg1.m_clearanceInfo.m_objectPoint[i];
      VDAddTempCfg(t1, false);
      VDClearLastTemp();
      //cout << "Witness Point::" << cfg1.m_clearanceInfo.m_objectPoint << endl;

      CfgType cfg2;
      CfgType incr;

      incr.GetRandomRay(length, _env, dm);
      cfg2 = cfg1 + incr;

      //VDComment("Cfg2");
      //VDAddTempCfg(cfg2, true);

      CfgType tick = cfg1, temp = cfg1;
      bool tickFree, tempFree = cfg1Free;
      int tickWitness, tempWitness = cfg1Witness;

      int nTicks;
      CfgType inter;
      inter.FindIncrement(cfg1, cfg2, &nTicks,
          100*_env->GetPositionRes(), 100*_env->GetOrientationRes());

      for(int i=1; i<nTicks; ++i) {

        tick += inter;

        VDAddTempCfg(tick, true);

        m_clearanceUtility.CollisionInfo(tick, tmp, _bb, tick.m_clearanceInfo);
        tickWitness = tick.m_clearanceInfo.m_nearestObstIndex;

        CfgType t1;
        for(size_t i = 0; i<CfgType::DOF(); ++i)
          t1[i] = cfg1.m_clearanceInfo.m_objectPoint[i];
        VDAddTempCfg(t1, false);

        VDClearLastTemp();
        VDClearLastTemp();

        //The closest obstacle is the same, check if the triangles which the
        //witness points belong to are adjacent and form a concave face
        if(tempWitness == tickWitness) {
          //Find the triangles which the witness points belong to first
          //assume obstacle multibodies have 1 body
          int tempID = FindTriangle(_env, tickWitness, temp);
          int tickID = FindTriangle(_env, tickWitness, tick);

          //if the triangle IDs have not been set, there has been an error in
          //the triangle computation
          assert(tempID != -1 && tickID != -1);

          //Check if two triangles are adjacent to each other
          //Find the common edge between two triangles
          GMSPolyhedron& polyhedron = _env->GetMultiBody(tickWitness)->GetBody(0)->GetPolyhedron();
          if(tempID != tickID) {
            pair<int, int> edge = polyhedron.m_polygonList[tempID].CommonEdge(polyhedron.m_polygonList[tickID]);
            Vector3d v0, v1, v2;
            if(edge.first != -1 && edge.second != -1) {  //If there is a common edge (v0, v1) between two triangle facets
              //Get vertex information for two facets
              v0 = polyhedron.m_vertexList[edge.first];
              v1 = polyhedron.m_vertexList[edge.second];
              for(vector<int>::iterator I = polyhedron.m_polygonList[tempID].m_vertexList.begin(); I != polyhedron.m_polygonList[tempID].m_vertexList.end(); I++) {
                if((*I != edge.first) && (*I != edge.second))
                  v2 = polyhedron.m_vertexList[*I];
              }
              //Find out which triangle is on the left and which is on the right
              Vector3d va = v1 - v0;  //Common edge (v0, v1)
              Vector3d vb = v2 - v0;  //The other edge (v0, v2)
              int left, right;
              //The face is on the left of the common edge if (va x vb).normal vector of the face > 0
              //If < 0, the face is on the right
              if(((va % vb) * polyhedron.m_polygonList[tempID].m_normal) > 0) {
                left = tempID;
                right = tickID;
              }
              else {
                left = tickID;
                right = tempID;
              }
              //Check if the two triangles form a concave face
              //If (normal vector of left) x (normal vector of right) is the
              //opposite direction of the common edge, they form a concave face
              if((((polyhedron.m_polygonList[left].m_normal) % (polyhedron.m_polygonList[right].m_normal)) * va) < 0){  //Concave
                //Find the medial axis
                tickFree = vc->IsValid(tick, _env, _stats, cdInfo, &callee)
                  && !vc->IsInsideObstacle(tick, _env, cdInfo);

                if(tempFree && tickFree) {
                  if((temp.m_clearanceInfo.m_minDist > tick.m_clearanceInfo.m_minDist) && _env->InBounds(temp, _bb)) {
                    _stats.IncNodesGenerated(this->GetNameAndLabel());
                    generated = true;
                    _cfgOut.push_back(temp);
                    tempFree = tickFree;
                    temp = tick;
                  }
                  else if((tick.m_clearanceInfo.m_minDist > temp.m_clearanceInfo.m_minDist) && _env->InBounds(tick, _bb)) {
                    _stats.IncNodesGenerated(this->GetNameAndLabel());
                    generated = true;
                    _cfgOut.push_back(tick);
                    tempFree = tickFree;
                    temp = tick;
                  }
                }
              }
              else {  //Convex
                tempWitness = tickWitness;
                temp = tick;
              }
            }
            else {  //no common edge, generate valid medial axis crossing
              tickFree = vc->IsValid(tick, _env, _stats, cdInfo, &callee)
                && !vc->IsInsideObstacle(tick, _env, cdInfo);
              if(tempFree && tickFree) {
                if((temp.m_clearanceInfo.m_minDist > tick.m_clearanceInfo.m_minDist) && _env->InBounds(temp, _bb)) {
                  _stats.IncNodesGenerated(this->GetNameAndLabel());
                  generated = true;
                  _cfgOut.push_back(temp);
                  tempFree = tickFree;
                  temp = tick;
                }
                else if((tick.m_clearanceInfo.m_minDist > temp.m_clearanceInfo.m_minDist) && _env->InBounds(tick, _bb)) {
                  _stats.IncNodesGenerated(this->GetNameAndLabel());
                  generated = true;
                  _cfgOut.push_back(tick);
                  tempFree = tickFree;
                  temp = tick;
                }
              }
            }
          }
        }
        else {  //tempWitness != tickWitness; the closest obstacle changes
          tickFree = vc->IsValid(tick, _env, _stats, cdInfo, &callee)
            && !vc->IsInsideObstacle(tick, _env, cdInfo);
          //Both temp and tick are valid, keep the one with larger clearance
          if(tempFree && tickFree) {
            if((temp.m_clearanceInfo.m_minDist > tick.m_clearanceInfo.m_minDist) && _env->InBounds(temp, _bb)) {
              _stats.IncNodesGenerated(this->GetNameAndLabel());
              generated = true;
              _cfgOut.push_back(temp);
              tempFree = tickFree;
              temp = tick;
            }
            else if((tick.m_clearanceInfo.m_minDist > temp.m_clearanceInfo.m_minDist) && _env->InBounds(tick, _bb)) {
              _stats.IncNodesGenerated(this->GetNameAndLabel());
              generated = true;
              _cfgOut.push_back(tick);
              tempFree = tickFree;
              temp = tick;
            }
          }
          //Either temp or tick is valid, keep the valid one
          else if(tempFree || tickFree) {
            if(tempFree && _env->InBounds(temp, _bb)) {
              _stats.IncNodesGenerated(this->GetNameAndLabel());
              generated = true;
              _cfgOut.push_back(temp);
              tempFree = tickFree;
              temp = tick;
            }
            else if(tickFree && _env->InBounds(tick, _bb)){
              _stats.IncNodesGenerated(this->GetNameAndLabel());
              generated = true;
              _cfgOut.push_back(tick);
              tempFree = tickFree;
              temp = tick;
            }
          }
        }
      }
      return generated;
    }

    int FindTriangle(Environment* _env, int _witness, const CfgType& _c) {
      StatClass* stat = this->GetMPProblem()->GetStatClass();
      stat->StartClock("FindTriangle");
      //Find the triangles which the witness points belong to first
      //assume obstacle multibodies have 1 body
      GMSPolyhedron& polyhedron = _env->GetMultiBody(_witness)->GetBody(0)->GetPolyhedron();
      Transformation& t = _env->GetMultiBody(_witness)->GetBody(0)->WorldTransformation();

      Vector3d witnessPoint = -t * _c.m_clearanceInfo.m_objectPoint;
      double minProjDist = 1e6;
      int id = -1;
      for(size_t i=0; i < polyhedron.m_polygonList.size(); ++i) {
      //for(size_t i=polyhedron.m_polygonList.size()-1; i >= 0; --i) {
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

        Vector3d v0 = polyhedron.m_vertexList[poly.m_vertexList[0]];
        Vector3d v1 = polyhedron.m_vertexList[poly.m_vertexList[1]];
        Vector3d v2 = polyhedron.m_vertexList[poly.m_vertexList[2]];

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
            << "tempWitnessPoint:: " << tempWitnessPoint << "\ttickWitnessPoint:: " << tickWitnessPoint << endl
            << "Normal:: " << poly.m_normal << endl
            << "p0:: " << p0 << "\tp1:: " << p1 << "\tp2:: " << p2 << endl
            << "ptemp:: " << ptemp << endl
            << "ptempIn:: " << PtInTriangle(p0, p1, p2, ptemp) << endl;
            */

          double u,v;
          bool inTri = PtInTriangle(p0, p1, p2, ptemp, u, v);

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

};

#endif

