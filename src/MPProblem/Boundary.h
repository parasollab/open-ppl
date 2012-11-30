#ifndef BOUNDARY_H_
#define BOUNDARY_H_

#include "Vector.h"
#include "Point.h"

#include "Utilities/IOUtils.h"

class Environment;

using namespace mathtool;

///\todo add MPBaseObject defautl constructor
class Boundary {
  public:
    enum parameter_type{TRANSLATIONAL,REVOLUTE,PRISMATIC};
    Boundary();
    Boundary(XMLNodeReader& _node);
    virtual ~Boundary();
    virtual double GetClearance(Vector3D _point3d) const = 0;
    virtual bool IfSatisfiesConstraints(Vector3D _point3d) const =0;
    virtual bool InBoundary(const Cfg& _cfg, Environment* _env) = 0;
    virtual bool operator==(const Boundary& _bb) const =0; 
    virtual Point3d GetRandomPoint() = 0; 
    double GetRandomValueInParameter(int _par);
    virtual bool IsInterSect(Boundary* _b){ return true;}//not implemented yet
    virtual Boundary* GetIntersect(Boundary* _b){return _b;}//not implemented yet
    virtual bool IsOverlap(Boundary* _b){return true;}//not implemented yet
    virtual Boundary* GetOverlap(Boundary* _b){return _b;}//not implemented yet
    virtual void Print(std::ostream& _os, char _rangeSep=':', char _parSep=';') const=0 ;
    const std::pair<double,double> GetRange(int _par) const;
    parameter_type GetType(int _par) const;
    void TranslationalScale(double _scaleFactor);
    void SetParameter(int _par, double _pFirst, double _pSecond);
    void SetRange(std::vector<double>& _ranges);
    virtual bool IfEnoughRoom(int _par, double _room) =0;
    int GetDOFs() const;
    int GetPosDOFs() const;
  protected:
    std::vector<parameter_type> m_parType;
    vector< std::pair<double,double> > m_jointLimits; 
    int m_posDOFs;
    int m_DOFs;
    std::vector< std::pair<double,double> > m_boundingBox;
  
  public:
#ifdef _PARALLEL

    void define_type(stapl::typer&) { }

#endif
};

#ifdef _PARALLEL
namespace stapl {


  /*

     template <typename Accessor>
     class proxy<Boundary, Accessor> 
     : public Accessor {
     private:
     friend class proxy_core_access;
     typedef Boundary target_t;

     public:
     explicit proxy(Accessor const& acc) : Accessor(acc) { }
  //operator target_t() const { return Accessor::read(); }
  proxy const& operator=(proxy const& rhs) { Accessor::write(rhs); return *this; }
  proxy const& operator=(target_t const& rhs) { Accessor::write(rhs); return *this;}
  double GetRandomValueInParameter(int _par) { return Accessor::invoke(&target_t::GetRandomValueInParameter, _par);}
  };*/


}
#endif

#endif /*_Boundary_h_*/
