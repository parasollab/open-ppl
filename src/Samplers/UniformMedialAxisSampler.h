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

    UniformMedialAxisSampler(Environment* _env = NULL, string _vcLabel = "", string _dmLabel = "", double _length = 0, 
        bool _useBoundary = false, const ClearanceUtility<MPTraits>& _clearanceUtility = ClearanceUtility<MPTraits>())
      : m_length(_length), m_useBoundary(_useBoundary), m_vcLabel(_vcLabel), m_dmLabel(_dmLabel), m_clearanceUtility(_clearanceUtility) {
        this->SetName("UniformMedialAxisSampler");
      }

    UniformMedialAxisSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem, _node), m_clearanceUtility(_problem, _node) {
      this->SetName("UniformMedialAxisSampler");
      ParseXML(_node);
    }

    ~UniformMedialAxisSampler() {}

    void ParseXML(XMLNodeReader& _node) {
      m_length = _node.numberXMLParameter("d", true, 0.0, 0.0, MAX_DBL, "generate line segment with length d");
      m_useBoundary = _node.boolXMLParameter("useBBX", true, false, "Use bounding box as obstacle");
      m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
      m_dmLabel =_node.stringXMLParameter("dmLabel", true, "default", "Distance Metric Method");

      _node.warnUnrequestedAttributes();
    }

    virtual void PrintOptions(ostream& _os) const {
      SamplerMethod<MPTraits>::PrintOptions(_os);
      m_clearanceUtility.PrintOptions(_os);
      _os << "\tlength = " << m_length << endl;
      _os << "\tuseBoundary = " << m_useBoundary << endl;
      _os << "\tvcLabel = " << m_vcLabel << endl;
      _os << "\tdmLabel = " << m_dmLabel << endl;
    }

    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb, StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut, vector<CfgType>& _cfgCol) {
      string callee(this->GetNameAndLabel() + "::SampleImpl()");
      CDInfo cdInfo;
      ValidityCheckerPointer vc = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      DistanceMetricPointer dm = this->GetMPProblem()->GetDistanceMetric(m_dmLabel);

      bool generated = false;
      int attempts = 0;
      bool cfg1Free;
      int cfg1Witness;

      double length = m_length;
      if(length == 0){
        if(_env != NULL)
          length = (_env->GetMultiBody(_cfgIn.GetRobotIndex()))->GetMaxAxisRange();
      }

      _stats.IncNodesAttempted(this->GetNameAndLabel());
      attempts++;
      //Generate first cfg
      CfgType cfg1 = _cfgIn;
      CfgType tmp;
      if(cfg1 == CfgType())
        cfg1.GetRandomCfg(_env, _bb);

      cfg1Free = (vc->IsValid(cfg1, _env, _stats, cdInfo, &callee)) && (!vc->IsInsideObstacle(cfg1, _env, cdInfo));
      m_clearanceUtility.CollisionInfo(cfg1, tmp, _bb, cfg1.m_clearanceInfo);
      cfg1Witness = cfg1.m_clearanceInfo.m_nearestObstIndex;

      CfgType cfg2;
      CfgType incr;
      double dist, r;

      incr.GetRandomRay(length, _env, dm);
      cfg2 = cfg1 + incr;

      //Scale the distance between c1 and c2
      Vector3d c1, c2, dir;
      c1[0] = cfg1[0];
      c1[1] = cfg1[1];
      c1[2] = cfg1[2];
      c2[0] = cfg2[0];
      c2[1] = cfg2[1];
      c2[2] = cfg2[2];
      dir = c2 - c1;
      dist = sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
      r = length/dist;
      cfg2 = cfg1 + incr*r;

      CfgType inter;
      CfgType tick = cfg1;
      int nTicks;
      double positionRes = _env->GetPositionRes(); 
      double orientationRes = _env->GetOrientationRes();
      bool tempFree = cfg1Free;
      int tempWitness = cfg1Witness;
      bool tickFree;
      int tickWitness;
      CfgType temp = cfg1;

      inter.FindIncrement(cfg1, cfg2, &nTicks, positionRes, orientationRes);
      for(int i=1; i<nTicks; i++) {
        tick += inter;
        m_clearanceUtility.CollisionInfo(tick, tmp, _bb, tick.m_clearanceInfo);
        tickWitness = tick.m_clearanceInfo.m_nearestObstIndex;

        //The closest obstacle is the same, check if the triangles which the
        //witness points belong to are adjacent and form a concave face
        if(tempWitness == tickWitness) {  
          //Find the triangles which the witness points belong to first
          GMSPolyhedron& polyhedron = _env->GetMultiBody(temp.GetRobotIndex())->GetBody(temp.GetRobotIndex())->GetPolyhedron();
          Vector3d tempWitnessPoint = temp.m_clearanceInfo.m_objectPoint;
          Vector3d tickWitnessPoint = tick.m_clearanceInfo.m_objectPoint;
          int tempID, tickID;  //polygon/triangle id
          for(int i=0; i<polyhedron.m_polygonList.size(); i++)
          {
            GMSPolygon& poly = polyhedron.m_polygonList[i];
            //Find the max dimension among the normal vector so we know which
            //two dimensions (except the max dimension) form the surface (in
            //order to call PtInTriangle function)
            double max = poly.m_normal[0];
            int maxDim = 0;
            if(poly.m_normal[1] > max) {
              max = poly.m_normal[1];
              maxDim = 1;
            }
            if(poly.m_normal[2] > max) {
              max = poly.m_normal[2];
              maxDim = 2;
            }
            Vector3d& v0 = polyhedron.m_vertexList[poly.m_vertexList[0]];
            Vector3d& v1 = polyhedron.m_vertexList[poly.m_vertexList[1]];
            Vector3d& v2 = polyhedron.m_vertexList[poly.m_vertexList[2]];
            //p0, p1, p2 are the vertices of the triangle
            Point2d p0(v0[(maxDim+1)%3], v0[(maxDim+2)%3]);
            Point2d p1(v1[(maxDim+1)%3], v1[(maxDim+2)%3]);
            Point2d p2(v2[(maxDim+1)%3], v2[(maxDim+2)%3]);
            Point2d ptemp(tempWitnessPoint[(maxDim+1)%3], tempWitnessPoint[(maxDim+2)%3]);
            Point2d ptick(tickWitnessPoint[(maxDim+1)%3], tickWitnessPoint[(maxDim+2)%3]);
            //Store the triangle id if the witness point belongs to it
            if(PtInTriangle(p0, p1, p2, ptemp)) {
              tempID = i;
            }
            if(PtInTriangle(p0, p1, p2, ptick)) {
              tickID = i;
            }
          }
          //Check if two triangles are adjacent to each other
          //Find the common edge between two triangles
          if(tempID != tickID) {
            pair<int, int> edge = polyhedron.m_polygonList[tempID].CommonEdge(polyhedron.m_polygonList[tickID]);
            Vector3d v0, v1, v2;
            if((edge.first != -1) && (edge.second != -1)) {  //If there is a common edge (v0, v1) between two triangle facets
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
              if((((polyhedron.m_polygonList[left].m_normal) % (polyhedron.m_polygonList[right].m_normal)) * va) > 0){  //Concave
                //Find the medial axis
                tickFree = (vc->IsValid(tick, _env, _stats, cdInfo, &callee)) && (!vc->IsInsideObstacle(tick, _env, cdInfo));
                if(tempFree && tickFree) {
                  _stats.IncNodesGenerated(this->GetNameAndLabel());
                  generated = true;
                  if((temp.m_clearanceInfo.m_minDist > tick.m_clearanceInfo.m_minDist) && _env->InBounds(temp, _bb)) {
                    _cfgOut.push_back(temp);
                    tempFree = tickFree;
                    temp = tick;
                  }
                  else if((tick.m_clearanceInfo.m_minDist > temp.m_clearanceInfo.m_minDist) && _env->InBounds(tick, _bb)) {
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
          }
        }
        else {  //tempWitness != tickWitness; the closest obstacle changes
          tickFree = (vc->IsValid(tick, _env, _stats, cdInfo, &callee)) && (!vc->IsInsideObstacle(tick, _env, cdInfo));
          //Both temp and tick are valid, keep the one with larger clearance
          if(tempFree && tickFree) {
            _stats.IncNodesGenerated(this->GetNameAndLabel());
            generated = true;
            if((temp.m_clearanceInfo.m_minDist > tick.m_clearanceInfo.m_minDist) && _env->InBounds(temp, _bb)) {
              _cfgOut.push_back(temp);
              tempFree = tickFree;
              temp = tick;
            }
            else if((tick.m_clearanceInfo.m_minDist > temp.m_clearanceInfo.m_minDist) && _env->InBounds(tick, _bb)) {
              _cfgOut.push_back(tick);
              tempFree = tickFree;
              temp = tick;
            }
          }
          //Either temp or tick is valid, keep the valid one
          else if(tempFree || tickFree) {
            _stats.IncNodesGenerated(this->GetNameAndLabel());
            generated = true;
            if(tempFree && _env->InBounds(temp, _bb)) {
              _cfgOut.push_back(temp);
              tempFree = tickFree;
              temp = tick;
            }
            else if(tickFree && _env->InBounds(tick, _bb)){
              _cfgOut.push_back(tick);
              tempFree = tickFree;
              temp = tick;
            }
          }
        }
      }
      return generated;
    }   
};

#endif

