#ifndef PMPL_MINKOWSKI_DISTANCE_H_
#define PMPL_MINKOWSKI_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "MPProblem/Environment/Environment.h"


////////////////////////////////////////////////////////////////////////////////
/// Minkowski distance is a generalized L-p norm where the exponents p and 1/p
/// each have separate values.
///
/// @todo Remove m_r2, which isn't part of the Minkowski difference and doesn't
///       have any valid use.
///
/// @todo Move the normalization option up to the base class.
///
/// @todo Separate the computation of orientation and joint distances.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class MinkowskiDistance : virtual public DistanceMetricMethod {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType  GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1. / 3,
        bool _normalize = false);

    MinkowskiDistance(XMLNode& _node);

    virtual ~MinkowskiDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name DistanceMetricMethod Overrides
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    virtual void ScaleCfg(double _length, Cfg& _c, const Cfg& _o)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    double PositionDistance(const Cfg& _c);
    double OrientationDistance(const Cfg& _c);

    ///@}
    ///@name Internal State
    ///@{

    double m_r1{3};          ///< For position part.
    double m_r2{3};          ///< For rotation part.
    double m_r3{1. / 3.};    ///< For calculating root.
    bool m_normalize{false}; ///< Normalize distance w.r.t. environment?

    ///@}
};

#endif
