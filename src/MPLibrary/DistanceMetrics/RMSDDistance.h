#ifndef PMPL_RMSD_DISTANCE_H_
#define PMPL_RMSD_DISTANCE_H_

#include "DistanceMetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// @todo Properly document. This looks like a very complex way to assess RMS
///       distance between body translations.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class RMSDDistance : virtual public DistanceMetricMethod {

  public:

    ///@name Local Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    RMSDDistance();
    RMSDDistance(XMLNode& _node);
    virtual ~RMSDDistance() = default;

    ///@}
    ///@name Distance Interface
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2);

    // Overrides
    virtual void ScaleCfg(double _length, Cfg& _c, const Cfg& _o) override;

  private:

    ///@name Helpers
    ///@{

    std::vector<Vector3d> GetCoordinatesForRMSD(const Cfg& _c);
    double RMSD(std::vector<Vector3d> _x, std::vector<Vector3d> _y, int _dim);

    ///@}

};

#endif
