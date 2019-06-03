#ifndef OBSTACLE_BASED_H_
#define OBSTACLE_BASED_H_

#include "ITPlacementMethod.h"

#include "TMPLibrary/TMPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class ObstacleBased : public ITPlacementMethod {

  public:

    ///@name Construction
    ///@{

    ObstacleBased(MPProblem* _problem);

    ObstacleBased(XMLNode& _node);

    ~ObstacleBased() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator) override;

    ///@}


  protected:

    void Fill();
    vector<vector<vector<double>>> GetObstaclesBounds();
    void TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg);
    bool IsInObstacle(vector<double> point, int method);
    bool IsPointValid(vector<double> point);
    vector<vector<double>> RefinePossibleLocations(vector<vector<double>> possibleLocations);

  private:

    double m_ballRadius, m_ballDiameter, m_precision, m_startCornerAngle, 
        m_endCornerAngle, m_stepCornerAngle, m_checksPerSide, m_refineMethod;
    Environment* m_environment;
    const Boundary* m_boundary;
    vector<Cfg> m_locations;
    int m_dimensions;
    std::map<size_t, vector<Vector3d>> m_obstacles;
    vector<vector<vector<double>>> m_obstaclesBounds;


};

#endif
