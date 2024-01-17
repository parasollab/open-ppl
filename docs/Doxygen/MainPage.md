Documentation for all classes can be found under "Classes" and "Files". Use case
examples for main components can be found under "Examples".

To use the Parasol Planning Library: [Parasol Planning Library Github Repository](parasollab.web.illinois.edu)

## About the Parasol Laboratory
The Parasol Laboratory is a focal point for research related to next-generation computing languages and systems and for the development of algorithms and applications that exploit these to solve computation and/or data intensive applications. Due to its application-centric focus, the Parasol Lab creates a uniquely favorable environment for multidisciplinary collaboration between systems and application developers.

Parasol software and systems projects include: the study and development of novel algorithmic, architecture and compiler techniques for the optimization of parallel and distributed systems, the design and implementation of compiler driven software productivity improvement tools, software verification, and performance modeling and prediction.

Parasol applications projects include: the development of optimized algorithms
for applications from domains such as computer-aided design (CAD), computational biology, computational geophysics, computational neuroscience, computational physics, robotics, and virtual reality.

Visit our website for more details: [Parasol Laboratory Webpage](parasollab.web.illinois.edu)

## Library Structure: 
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
 - [Metrics](\ref MetricMethod): Track various aspects motion planning algorithms.
 <!-- - [Path Modifiers](\ref PathModifierMethod): Modifiers for paths, after they are found.  -->

 If you are looking for something specific, `Files->File List` provides a list of all our files in source code hierarchy, and `Data Structures->Data Structures Index` provides a list of all data structures in the library. 

## Contact Us
Email us at: parasol@illinois.edu
