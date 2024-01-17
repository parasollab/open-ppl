#ifndef PMPL_PATH_EVALUATOR_H
#define PMPL_PATH_EVALUATOR_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This class calculates various metrics for the path found by another MapEval.
/// Note: this class does not run Dijkstra's.  
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class PathEvaluator : public MapEvaluatorMethod {

    public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID               VID;


    ///@}
    ///@name Construction
    ///@{

    PathEvaluator();

    PathEvaluator(XMLNode& _node);

    virtual ~PathEvaluator() = default;

    ///@}

    ///@name MPBaseObject Interface
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluatorMethod Interface
    ///@{

    virtual bool operator()() override;

    ///@}

    protected:

    private:

    /// Returns a vector of length 3:
    /// [0] is the min clearance
    /// [1] is the max clearance
    /// [2] is the average clearance.  
    std::vector<double> GetClearanceStats(const Path* path);

    /// Returns the total path length (using the distance metric).
    double GetPathLength(const Path* path);

    /// Variables fill by XML:
    std::string m_cuLabel{}; // Clearance Utility (MPTool)
    std::string m_ievcLabel{}; // Intermediates Edge Validity Checker
    std::string m_dmLabel{}; // Distance Metric 

    // Why copy/paste when you could write a bunch of helper functions instead?
    // Each of these functions adds (key, value) to this's stat class
    // and prints them to std::cout. 
    // You can find statistics in the .stat output file. 
    void AddToStats(std::string _key, double _value);
    inline void AddToStats(std::string _key, int _value) { AddToStats(_key, (double)_value); };
    inline void AddToStats(std::string _key, bool _value) { AddToStats(_key, (double)_value); };
    inline void AddToStats(std::string _key, size_t _value) { AddToStats(_key, (double)_value); };


};

#endif
