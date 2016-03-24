#ifndef SURFACE_GRID_SAMPLER_H_
#define SURFACE_GRID_SAMPLER_H_

#ifdef PMPCfgSurface

#include "SamplerMethod.h"

#include "Environment/SurfaceMultiBody.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup Samplers
/// @brief Sample surface configurations on a grid throughout the surfaces
/// @tparam MPTraits Motion planning universe
///
/// numSurf<-GetNavigableSurfaceCount()
/// for i:-1 to numSurf
///   if i>=0
///     poli= getSurfacePolyhedron
///     bounds <- poli.bounds
///     for x:boundsMinx to boundsMaxx
///        for z:boundsMinz to boundsMaxz
///           p<-point3d(x,getH(x,z),z)
///           if o is valid
///              add p to graph
///  if i==-1
///    bounds <- env.bounds
///    for x:boundsMinx to boundsMaxx
///    for z:boundsMinz to boundsMaxz
///      p<-point3d(x,0,z)
///      if o is valid
///        add p to graph
////////////////////////////////////////////////////////////////////////////////
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
    SurfaceGridSampler(string _vcLabel="", double _dx=5.0, double _dz=5.0)
      : m_vcLabel(_vcLabel), m_dx(_dx), m_dz(_dz) {
      this->SetName("SurfaceGridSampler");
    }

    //Constructor
    SurfaceGridSampler(MPProblemType* _problem, XMLNode& _node)
      : SamplerMethod<MPTraits>(_problem,_node) {
      this->SetName("SurfaceGridSampler");
      ParseXML(_node);
    }

    ~SurfaceGridSampler() {}

    //Reading XML node
    void ParseXML(XMLNode& _node){
    m_vcLabel = _node.Read("vcLabel", true, "", "Validity Test Method");
    m_dx = _node.Read("dx",true,1.0,0.1,20.0,"delta x");
    m_dz = _node.Read("dz",true,1.0,0.1,20.0,"delta z");
    }

    virtual void Print(ostream& _out) const {
      SamplerMethod<MPTraits>::Print(_out);
      _out<< "\tm_vcLabel= " << m_vcLabel << endl;
      _out<< "\tdx= " << m_dx << "\tdz= " << m_dz << endl;
    }

    //Generates grid cfgs over each navigable surface
    virtual bool Sampler(CfgType& _cfg, shared_ptr<Boundary> _boundary,
        vector<CfgType>& _result, vector<CfgType>& _collision) {
      Environment* env = this->GetEnvironment();
      string callee(this->GetNameAndLabel());
      callee += "::SampleImpl()";
      ValidityCheckerPointer vcp = this->GetValidityChecker(m_vcLabel);
      int numSurfaces=  env->NumSurfaces(); //Number of navigable surface
      for(int i = -1; i<numSurfaces; i++) {
        //For the navigable surfaces not including the surface -1 (ground)
        if(i>=0) {
          //Get navigable surface by number and get the polyhedron of that surface
          shared_ptr<SurfaceMultiBody> surfi = env->GetSurface((size_t) i);
          shared_ptr<FixedBody> fb = surfi->GetFixedBody(0);
          GMSPolyhedron& polyhedron = fb->GetWorldPolyhedron();
          //Obtain the bondaries of the surface
          const double* boundaries = surfi->GetBoundingBox();
          //Try to add point by point in the greed to the graph
          for(double itdx=boundaries[0]; itdx<boundaries[1]; itdx+=m_dx) {
            for(double itdz=boundaries[4]; itdz<boundaries[5]; itdz+=m_dz) {
              bool isValSurf=true;
              //Generate a point with the values of the iterators of the loops
              Point2d pt(itdx,itdz);
              //Get the height of the point defined before
              double tH = polyhedron.HeightAtPt(pt, isValSurf);
              //If there is a valid height fpr the point
              if( isValSurf ) {
                CfgType tmp;
                //set the surface cfg to add
                tmp.SetSurfaceID(i);
                tmp.SetPos(pt);
                tmp.SetHeight(tH);
                //Validate surface cfg
                bool isValid = vcp->IsValid(tmp, callee);
                if(isValid) {
                  //Add valid surface cfg
                  _result.push_back(tmp);
                }
                else {
                  _collision.push_back(tmp);
                }
              }
            }
          }
        }
        //Ground grid sampling
        else if(i==-1) {
          //Get environment boundaries for axis x and z
          pair<double, double> dxx = _boundary->GetRange(0);
          pair<double, double> dxz = _boundary->GetRange(2);
          for(double itdx=dxx.first; itdx<dxx.second; itdx+=m_dx) {
            for(double itdz=dxz.first; itdz<dxz.second; itdz+=m_dz) {
              Point2d pt(itdx,itdz);
              CfgType tmp;
              tmp.SetSurfaceID(i);
              tmp.SetPos(pt);
              //Height is 0 for this points
              tmp.SetHeight(0);
              bool isValid = vcp->IsValid(tmp, callee);
              if(isValid) {
                _result.push_back(tmp);
              }
              else {
                _collision.push_back(tmp);
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
