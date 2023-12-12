#ifndef PMPL_BINARY_LP_SWEPT_DISTANCE_H_
#define PMPL_BINARY_LP_SWEPT_DISTANCE_H_

#include "LPSweptDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// BinaryLPSwept Distance is LPSwept Distance with positon and orientation
/// resolutions initially set and then doubled in granularity until distance
/// converges, the environmental resolutions are met, or maxAttempts are reached
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
class BinaryLPSweptDistance : virtual public LPSweptDistance {

  public:

    ///@name Local Types
    ///@{

    

    ///@}
    ///@name Construction
    ///@{

    BinaryLPSweptDistance(string _lp = "sl", double _posRes = .1,
        double _oriRes = .1, double _tolerance = .01, int _maxAttempts = 100,
        bool _bbox = false);
    BinaryLPSweptDistance(XMLNode& _node);

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(ostream& _os) const override;

    ///@}
    ///@name LPSweptDistance Overrides
    ///@{

    virtual double Distance(const Cfg& _c1, const Cfg& _c2) override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    double m_tolerance;
    int m_maxAttempts;

    ///@}
};

#endif
