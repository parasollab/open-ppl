/*
 * SurfaceGridSampler.h
 * Grid Sampling in the surface.
 *
 * numSurf<-GetNavigableSurfaceCount()
 * for i:-1 to numSurf
 *   if i>=0
 *     poli= getSurfacePolyhedron
 *     bounds <- poli.bounds
 *     for x:boundsMinx to boundsMaxx
 *        for z:boundsMinz to boundsMaxz
 *           p<-point3d(x,getH(x,z),z)
 *           if o is valid
 *              add p to graph
 *  if i==-1
 *    bounds <- env.bounds
 *    for x:boundsMinx to boundsMaxx
 *    for z:boundsMinz to boundsMaxz
 *      p<-point3d(x,0,z)
 *      if o is valid
 *        add p to graph
 * */

#ifndef SURFACEGRIDSAMPLER_H_
#define SURFACEGRIDSAMPLER_H_

#ifdef PMPCfgSurface

#include "SamplerMethod.h"


template<class MPTraits>
class SurfaceGridSampler : public SamplerMethod<MPTraits> {

  private:
    std::string m_vcLabel;
    double m_dx; //Delta x
    double m_dz; //Delta y

  public:
    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::MPProblemType MPProblemType;
    typedef typename MPProblemType::DistanceMetricPointer DistanceMetricPointer;
    typedef typename MPProblemType::ValidityCheckerPointer ValidityCheckerPointer;

    //Constructor
    SurfaceGridSampler(string _vcLabel="", double _dx=5.0, double _dz=5.0) : m_vcLabel(_vcLabel), m_dx(_dx), m_dz(_dz) {
      this->SetName("SurfaceGridSampler");
    }

    //Constructor
    SurfaceGridSampler(MPProblemType* _problem, XMLNodeReader& _node) : SamplerMethod<MPTraits>(_problem,_node) {
      this->SetName("SurfaceGridSampler");
      ParseXML(_node);
    }

    ~SurfaceGridSampler() {}

    //Reading XML node
    void ParseXML(XMLNodeReader& _node){
    m_vcLabel = _node.stringXMLParameter("vcLabel", true, "", "Validity Test Method");
    m_dx = _node.numberXMLParameter("dx",true,1.0,0.1,20.0,"delta x");
    m_dz = _node.numberXMLParameter("dz",true,1.0,0.1,20.0,"delta z");
    }

    virtual void PrintOptions(ostream& _out) const {
      SamplerMethod<MPTraits>::PrintOptions(_out);
      _out<< "\tm_vcLabel= " << m_vcLabel << endl;
      _out<< "\tdx= " << m_dx << "\tdz= " << m_dz << endl;
    }

    //Generates grid cfgs over each navigable surface
    virtual bool Sampler(Environment* _env, shared_ptr<Boundary> _bb,
        StatClass& _stats, CfgType& _cfgIn, vector<CfgType>& _cfgOut,
        vector<CfgType>& _cfgCol) {
      string callee(this->GetName());
      callee += "::SampleImpl()";
      ValidityCheckerPointer vcp = this->GetMPProblem()->GetValidityChecker(m_vcLabel);
      int numSurfaces=  _env->GetNavigableSurfacesCount(); //Number of navigable surface
      for(int i=-1; i<numSurfaces; i++) {
        //For the navigable surfaces not including the surface -1 (ground)
        if(i>=0){
          //Get navigable surface by number and get the polyhedron of that surface
          shared_ptr<MultiBody> surfi = _env->GetNavigableSurface((size_t) i);
          shared_ptr<FixedBody> fb = surfi->GetFixedBody(0);
          GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();
          //Obtain the bondaries of the surface
          const double* boundaries = surfi->GetBoundingBox();
          //Try to add point by point in the greed to the graph
          for(double itdx=boundaries[0]; itdx<boundaries[1]; itdx+=m_dx){
            for(double itdz=boundaries[4]; itdz<boundaries[5]; itdz+=m_dz){
              bool isValSurf=true;
              //Generate a point with the values of the iterators of the loops
              Point2d pt(itdx,itdz);
              //Get the height of the point defined before
              double tH = polyhedron.HeightAtPt(pt, isValSurf);
              //If there is a valid height fpr the point
              if( isValSurf ){
                CfgType tmp;
                //set the surface cfg to add
                tmp.SetSurfaceID(i);
                tmp.SetPos(pt);
                tmp.SetHeight(tH);
                //Validate surface cfg
                bool isValid = vcp->IsValid(tmp, callee);
                if(isValid){
                  //Add valid surface cfg
                  _cfgOut.push_back(tmp);
                }else{
                  _cfgCol.push_back(tmp);
                }
              }
            }
          }
        }
        //Ground grid sampling
        else if(i==-1){
          //Get environment boundaries for axis x and z
          pair<double, double> dxx = _env->GetRange(0, _bb);
          pair<double, double> dxz = _env->GetRange(2, _bb);
          for(double itdx=dxx.first; itdx<dxx.second; itdx+=m_dx){
            for(double itdz=dxz.first; itdz<dxz.second; itdz+=m_dz){
              Point2d pt(itdx,itdz);
              CfgType tmp;
              tmp.SetSurfaceID(i);
              tmp.SetPos(pt);
              //Height is 0 for this points
              tmp.SetHeight(0);
              bool isValid = vcp->IsValid(tmp, callee);
              if(isValid){
                _cfgOut.push_back(tmp);
              }else{
                _cfgCol.push_back(tmp);
              }
            }
          }
        }
      }
      return true;
    }
};

#endif
#endif


