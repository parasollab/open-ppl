#ifndef BALL_FILLING_H_
#define BALL_FILLING_H_

#include "PlacementMethod.h"

#include "MPLibrary/MPTools/InteractionTemplate.h"
#include "Utilities/XMLNode.h"

class BallFilling : public PlacementMethod {

  public:

    ///@name Construction
    ///@{

    BallFilling(MPProblem* _problem); 

    BallFilling(MPProblem* _problem, XMLNode& _node);

    ~BallFilling() = default;

    ///@}
    ///@name Interface
    ///@{

    void virtual PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator);

    ///@}

    void CreateBalls();
    bool isObstacle2D(double circleX, double circleY, double radius, double topLeftRectX, double topLeftRectY, double rectWidth, double rectHeight);
    bool checkCircleCollide(double x1, double y1, double r1, double x2, double y2, double r2);
    vector<vector<double>> GetObstaclesVertices();


  private:

    const Environment* m_environment;
    const Boundary* m_boundary;
    vector<Cfg> m_locations;
    double m_radius;
    int m_dimension;
    double m_step;
    double m_precision;
    std::map<size_t, std::vector<Vector3d>> m_obstacles;

};

#endif
