# Parasol Planning Library (PPL)

### Structure: 
\ref MPBaseObject is our base type. 

We also have:
 - [Motion Planning Strategies](\ref MPStrategyMethod): Motion planning methods, fully implemented
 - [Samplers](\ref SamplerMethod): Random samplers for motion planning.
 - [Extenders](\ref ExtenderMethod): For RRT tree extensions.
 - [Local Planners](\ref LocalPlannerMethod): For local planning for PRMs.
 - [Distance Metrics](\ref DistanceMetricMethod): Various ways of measuring distance. 
 - [Connectors](\ref ConnectorMethod): For connecting samples in c-space.
 - [Neighborhood Finders](\ref NeighborhoodFinderMethod): For finding nearest neighbors  based on various criterion.
 - [Validity Checkers](\ref ValidityCheckerMethod): For checking the validity of points in c-space.
 - [Edge Validity Checkers](\ref EdgeValidityCheckerMethod): Whole-edge checking in c-space.
 - [Map Evaluators](\ref MapEvaluatorMethod): Evaluators to decide when to stop sampling. 
 - [Query Evaluators](\ref QueryMethod): Query techniques on graphs. Most run Dijkstra's algorithm. 
 - [Metrics](\ref MetricMethod): ???
 - [Path Modifiers](\ref PathModifierMethod): Modifiers for paths, after they are found. 